HiGHS_optimizer_small_gap = JuMP.optimizer_with_attributes(
    HiGHS.Optimizer,
    "time_limit" => 100.0,
    "random_seed" => 12345,
    "mip_rel_gap" => 0.001,
    "log_to_console" => false,
)

#Utility function for testing for adding outage data to generators and reserves:
function add_outages_to_systems!(
    systems::Vector{System},
    outage_specifications::Vector{
        @NamedTuple{
            outage_generators::Vector{String},
            responding_reserves::Dict{DataType, String},
        }
    },
)
    for outage_specification in outage_specifications
        outage_generators = outage_specification.outage_generators
        responding_reserves = outage_specification.responding_reserves
        transition_data = GeometricDistributionForcedOutage(;
            mean_time_to_recovery=10,  # Units of hours - This value does not have any influence for G-1 formulation
            outage_transition_probability=1.0,
        )
        for sys in systems
            for gen_name in outage_generators
                gen = get_component(PSY.Generator, sys, gen_name)
                add_supplemental_attribute!(sys, gen, transition_data)
            end
            for (reserve, reserve_name) in responding_reserves
                reserve_product = get_component(reserve, sys, reserve_name)
                add_supplemental_attribute!(sys, reserve_product, transition_data)
            end
        end
    end
end

function _get_reserve_name_for_results(reserve_type::DataType, reserve_name::String)
    a = string(Base.typename(reserve_type).wrapper)
    b = reserve_type.parameters[1]
    c = reserve_name
    return "__" * string(a) * "__" * string(b) * "__" * string(c)
end

@testset "10 bus; RampReserveWithDeliverabilityConstraints + AreaPTDFPowerModel (reference): separate reserves" begin
    sys = build_system(PSISystems, "two_area_pjm_DA"; add_reserves=true)
    # Make feasible by ignoring ramp limits: 
    for g in get_components(ThermalStandard, sys)
        set_ramp_limits!(g, nothing)
    end
    transform_single_time_series!(sys, Hour(24), Hour(1))

    # Make Sundance must run with a minimum active power: 
    set_must_run!(get_component(ThermalStandard, sys, "Sundance_1"), true)
    set_active_power_limits!(
        get_component(ThermalStandard, sys, "Sundance_1"),
        (min=1.0, max=2.0),
    )
    set_must_run!(get_component(ThermalStandard, sys, "Sundance_2"), true)
    set_active_power_limits!(
        get_component(ThermalStandard, sys, "Sundance_2"),
        (min=1.0, max=2.0),
    )

    outages_specifications = [
        (
            outage_generators=["Sundance_1"],
            responding_reserves=Dict(PSY.VariableReserve{ReserveUp} => "Reserve1_1"),
        )
        (
            outage_generators=["Sundance_2"],
            responding_reserves=Dict(PSY.VariableReserve{ReserveUp} => "Reserve1_2"),
        )
    ]
    add_outages_to_systems!([sys], outages_specifications)

    template = ProblemTemplate(NetworkModel(AreaPTDFPowerModel))
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, DeviceModel(MonitoredLine, StaticBranch))  # Only assign Monitored Line to subsystem A template
    set_device_model!(template, Line, StaticBranch)
    set_service_model!(
        template,
        ServiceModel(
            VariableReserve{ReserveUp},
            RampReserveWithDeliverabilityConstraints,
            "Reserve1_1",
        ),
    )
    set_service_model!(
        template,
        ServiceModel(
            VariableReserve{ReserveUp},
            RampReserveWithDeliverabilityConstraints,
            "Reserve1_2",
        ),
    )
    problem = DecisionModel(
        template,
        sys;
        name="UC_Subsystem",
        optimizer=HiGHS_optimizer_small_gap,
    )

    build_out = build!(problem; output_dir=mktempdir())
    solve!(problem)
    res = OptimizationProblemResults(problem)

    post_contingency_deployment_1 = read_variable(
        res,
        "PostContingencyActivePowerReserveDeploymentVariable__VariableReserve__ReserveUp__Reserve1_1",
    )
    post_contingency_deployment_2 = read_variable(
        res,
        "PostContingencyActivePowerReserveDeploymentVariable__VariableReserve__ReserveUp__Reserve1_2",
    )
    # Sum of the reserve deployments is equivalent to the outaged generator: 
    for post_contingency_deployment in
        [post_contingency_deployment_1, post_contingency_deployment_2]
        for i in unique(post_contingency_deployment[!, :DateTime])
            @test isapprox(
                sum(
                    filter(row -> row.DateTime == i, post_contingency_deployment)[
                        :,
                        "value",
                    ],
                ),
                100.0,
            )
        end
    end

    flows = read_expression(res, "PTDFBranchFlow__Line", table_format=TableFormat.WIDE)
    post_contingency_flows_1 = read_expression(
        res,
        "PostContingencyBranchFlow__VariableReserve__ReserveUp__Reserve1_1",
    )
    post_contingency_flows_2 = read_expression(
        res,
        "PostContingencyBranchFlow__VariableReserve__ReserveUp__Reserve1_2",
    )
    for line_name in get_name.(get_components(Line, sys))
        flow = flows[:, line_name]
        post_contingency_flow_1 =
            filter(row -> row.name2 == line_name, post_contingency_flows_1)[:, :value]
        post_contingency_flow_2 =
            filter(row -> row.name2 == line_name, post_contingency_flows_2)[:, :value]
        # Post contingency flow in the area of the outage is different, post contingency flow from the other area is the same:
        if occursin("_1", line_name)
            @test all(isapprox.(flow, post_contingency_flow_2))
            @test !all(isapprox.(flow, post_contingency_flow_1))
        elseif occursin("_2", line_name)
            @test all(isapprox.(flow, post_contingency_flow_1))
            @test !all(isapprox.(flow, post_contingency_flow_2))
        end
    end
end

@testset "10 bus; RampReserveWithDeliverabilityConstraints + SplitAreaPTDFPowerModel: separate reserves" begin
    sys = build_system(PSISystems, "two_area_pjm_DA"; add_reserves=true)
    # Make feasible by ignoring ramp limits: 
    for g in get_components(ThermalStandard, sys)
        set_ramp_limits!(g, nothing)
    end
    transform_single_time_series!(sys, Hour(24), Hour(1))

    # Make Sundance must run with a minimum active power: 
    set_must_run!(get_component(ThermalStandard, sys, "Sundance_1"), true)
    set_active_power_limits!(
        get_component(ThermalStandard, sys, "Sundance_1"),
        (min=1.0, max=2.0),
    )
    set_must_run!(get_component(ThermalStandard, sys, "Sundance_2"), true)
    set_active_power_limits!(
        get_component(ThermalStandard, sys, "Sundance_2"),
        (min=1.0, max=2.0),
    )

    outages_specifications = [
        (
            outage_generators=["Sundance_1"],
            responding_reserves=Dict(PSY.VariableReserve{ReserveUp} => "Reserve1_1"),
        )
        (
            outage_generators=["Sundance_2"],
            responding_reserves=Dict(PSY.VariableReserve{ReserveUp} => "Reserve1_2"),
        )
    ]
    add_outages_to_systems!([sys], outages_specifications)

    area_subsystem_map = Dict("Area1" => "a", "Area2" => "b")
    make_subsystems!(sys, area_subsystem_map)
    r1_1 = get_component(VariableReserve, sys, "Reserve1_1")
    add_component_to_subsystem!(sys, "a", r1_1)
    r1_2 = get_component(VariableReserve, sys, "Reserve1_2")
    add_component_to_subsystem!(sys, "b", r1_2)

    # From bus determines subsystem of ACBranch
    a_buses = get_components(ACBus, sys; subsystem_name="a")
    b_buses = get_components(ACBus, sys; subsystem_name="b")
    for b in get_components(ACTransmission, sys)
        from_bus = get_from(get_arc(b))
        if from_bus ∈ a_buses
            add_component_to_subsystem!(sys, "a", b)
        elseif from_bus ∈ b_buses
            add_component_to_subsystem!(sys, "b", b)
        end
    end
    template = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel), ["a", "b"])
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, DeviceModel(MonitoredLine, StaticBranch))
    set_device_model!(template, Line, StaticBranch)
    set_service_model!(
        template,
        ServiceModel(
            VariableReserve{ReserveUp},
            RampReserveWithDeliverabilityConstraints,
            "Reserve1_1",
        ),
        "a",
    )
    set_service_model!(
        template,
        ServiceModel(
            VariableReserve{ReserveUp},
            RampReserveWithDeliverabilityConstraints,
            "Reserve1_2",
        ),
        "b",
    )
    problem = DecisionModel(
        MultiRegionProblem,
        template,
        sys;
        name="UC_Subsystem",
        optimizer=HiGHS_optimizer_small_gap,
    )

    build_out = build!(problem; output_dir=mktempdir())
    solve!(problem)
    res = OptimizationProblemResults(problem)
    post_contingency_deployment_1 = get_variable_values(res)[PSI.VariableKey{
        PostContingencyActivePowerReserveDeploymentVariable,
        VariableReserve{ReserveUp},
    }(
        "Reserve1_1",
    )]
    post_contingency_deployment_2 = get_variable_values(res)[PSI.VariableKey{
        PostContingencyActivePowerReserveDeploymentVariable,
        VariableReserve{ReserveUp},
    }(
        "Reserve1_2",
    )]
    # Sum of the reserve deployments is equivalent to the outaged generator: 
    for post_contingency_deployment in
        [post_contingency_deployment_1, post_contingency_deployment_2]
        for i in unique(post_contingency_deployment[!, :time_index])
            @test isapprox(
                sum(
                    filter(row -> row.time_index == i, post_contingency_deployment)[
                        :,
                        "value",
                    ],
                ),
                1.0,
            )
        end
    end

    flows = get_expression_values(res)[PSI.ExpressionKey{PTDFBranchFlow, Line}("")]
    post_contingency_flows_1 = get_expression_values(res)[PSI.ExpressionKey{
        PostContingencyBranchFlow,
        VariableReserve{ReserveUp},
    }(
        "Reserve1_1",
    )]
    post_contingency_flows_2 = get_expression_values(res)[PSI.ExpressionKey{
        PostContingencyBranchFlow,
        VariableReserve{ReserveUp},
    }(
        "Reserve1_2",
    )]
    for line_name in get_name.(get_components(Line, sys))
        flow = filter(row -> row.name == line_name, flows)[:, :value]
        post_contingency_flow_1 =
            filter(row -> row.name2 == line_name, post_contingency_flows_1)[:, :value]
        post_contingency_flow_2 =
            filter(row -> row.name2 == line_name, post_contingency_flows_2)[:, :value]
        # Post contingency flow in the area of the outage is different
        if occursin("_1", line_name)
            @test !all(isapprox.(flow, post_contingency_flow_1; atol=1e-6))
        elseif occursin("_2", line_name)
            @test !all(isapprox.(flow, post_contingency_flow_2; atol=1e-6))
        end
    end
end

@testset "RTS multi-stage sim w/ RampReserveWithDeliverabilityConstraints and SplitAreaPTDFPowerModel" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    outages_specifications = [(
        outage_generators=["313_CC_1"],
        responding_reserves=Dict(PSY.VariableReserve{ReserveUp} => "Spin_Up_R3"),
    )]
    add_outages_to_systems!([sys, sys2], outages_specifications)
    results, sim = run_rts_multi_stage_decomposition_simulation(
        [sys, sys2];
        NT=5,
        mode="vertical",
        monitored_line_formulations=[StaticBranchUnbounded, StaticBranchUnbounded],
        use_emulator=false,
        add_reserves=true,
        in_memory=true, # fails with in_memory = false
    )
    results_uc0 = get_decision_problem_results(results, "UC0")
    results_uc_subsystem = get_decision_problem_results(results, "UC_Subsystem")
    active_power = read_realized_variable(
        results_uc_subsystem,
        "ActivePowerVariable__ThermalStandard";
        table_format=TableFormat.WIDE,
    )
    # Test sum of the responding reserve deployments is equivalent to the total outaged generation: 
    for outages_specification in outages_specifications
        outage_generators = outages_specification.outage_generators
        responding_reserves = outages_specification.responding_reserves
        for i in unique(active_power[!, :DateTime])
            total_reserve_deployment = 0.0
            total_lost_generation = 0.0
            for (reserve_type, reserve_name) in responding_reserves
                # NOTE: workaround because read_realized_variable broken for 3D results: https://github.com/NREL-Sienna/PowerSimulations.jl/issues/1390
                post_contingency_deployment = read_variable(
                    results_uc_subsystem,
                    "PostContingencyActivePowerReserveDeploymentVariable" *
                    _get_reserve_name_for_results(reserve_type, reserve_name),
                )[DateTime("2020-01-01T00:00:00")]
                total_reserve_deployment += sum(
                    filter(row -> row.DateTime == i, post_contingency_deployment)[
                        :,
                        "value",
                    ],
                )
            end
            for generator in outage_generators
                total_lost_generation +=
                    filter(row -> row.DateTime == i, active_power)[1, generator]
            end
            @test isapprox(total_lost_generation, total_reserve_deployment)
        end
    end
end
