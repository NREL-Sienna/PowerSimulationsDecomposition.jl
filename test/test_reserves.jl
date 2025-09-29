@testset "Test adding reserves to sub-problems" begin
    template = MultiProblemTemplate(
        NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true),
        ["a", "b"],
    )
    service_model = ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "test")
    set_service_model!(template, service_model)
    @test !isempty(template.sub_templates["a"].services)
    @test !isempty(template.sub_templates["b"].services)

    template = MultiProblemTemplate(
        NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true),
        ["a", "b"],
    )
    set_service_model!(template, VariableReserve{ReserveUp}, RangeReserve)
    @test !isempty(template.sub_templates["a"].services)
    @test !isempty(template.sub_templates["b"].services)
end

# TODO -unexpected change in number of variables between psy versions.
#psy4->psy5 adds 36 variables to subproblem a 
#psy4 -> psy5 adds 72 variables to subproblem b  
@testset "MOI test - w/out reserves" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "b")
    make_subsystems!(sys, area_subsystem_map)
    template = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel), ["a", "b"])
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    problem = DecisionModel(
        MultiRegionProblem,
        template,
        sys;
        name="UC_Subsystem",
        optimizer=optimizer_with_attributes(HiGHS.Optimizer),
    )
    build_out = build!(problem; output_dir=mktempdir())
    @test build_out == PowerSimulations.ModelBuildStatus.BUILT
    jump_problem_dict = get_jump_models(problem)
    moi_tests(jump_problem_dict["a"], 10470, 0, 1728, 864, 2640, true)
    moi_tests(jump_problem_dict["b"], 17436, 0, 3456, 1728, 5280, true)
end

@testset "MOI test - reserves in A" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    for (k, v) in get_contributing_device_mapping(sys)
        s = v.service
        if get_name(s) ∈ ["Reg_Up", "Reg_Down"]
            vec_d = v.contributing_devices
            for d in vec_d
                remove_service!(d, s)
            end
        end
    end
    remove_component!(sys, get_component(VariableReserve, sys, "Reg_Up"))
    remove_component!(sys, get_component(VariableReserve, sys, "Reg_Down"))
    area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "b")
    make_subsystems!(sys, area_subsystem_map)
    r1 = get_component(VariableReserve, sys, "Spin_Up_R1")
    add_component_to_subsystem!(sys, "a", r1)
    template = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel), ["a", "b"])
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_service_model!(template, VariableReserve{ReserveUp}, RangeReserve)
    problem = DecisionModel(
        MultiRegionProblem,
        template,
        sys;
        name="UC_Subsystem",
        optimizer=optimizer_with_attributes(HiGHS.Optimizer),
    )
    build_out = build!(problem; output_dir=mktempdir())
    @test build_out == PowerSimulations.ModelBuildStatus.BUILT
    jump_problem_dict = get_jump_models(problem)
    moi_tests(jump_problem_dict["a"], 11286, 0, 1728, 912, 2640, true)
    moi_tests(jump_problem_dict["b"], 17436, 0, 3456, 1728, 5280, true)
end

@testset "MOI test - reserves in B" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    for (k, v) in get_contributing_device_mapping(sys)
        s = v.service
        if get_name(s) ∈ ["Reg_Up", "Reg_Down"]
            vec_d = v.contributing_devices
            for d in vec_d
                remove_service!(d, s)
            end
        end
    end
    remove_component!(sys, get_component(VariableReserve, sys, "Reg_Up"))
    remove_component!(sys, get_component(VariableReserve, sys, "Reg_Down"))
    area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "b")
    make_subsystems!(sys, area_subsystem_map)
    r2 = get_component(VariableReserve, sys, "Spin_Up_R2")
    add_component_to_subsystem!(sys, "b", r2)
    template = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel), ["a", "b"])
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_service_model!(template, VariableReserve{ReserveUp}, RangeReserve)
    problem = DecisionModel(
        MultiRegionProblem,
        template,
        sys;
        name="UC_Subsystem",
        optimizer=optimizer_with_attributes(HiGHS.Optimizer),
    )
    build_out = build!(problem; output_dir=mktempdir())
    @test build_out == PowerSimulations.ModelBuildStatus.BUILT
    jump_problem_dict = get_jump_models(problem)
    moi_tests(jump_problem_dict["a"], 10470, 0, 1728, 864, 2640, true)
    moi_tests(jump_problem_dict["b"], 18348, 0, 3456, 1776, 5280, true)
end

HiGHS_optimizer_small_gap = JuMP.optimizer_with_attributes(
    HiGHS.Optimizer,
    "time_limit" => 100.0,
    "random_seed" => 12345,
    "mip_rel_gap" => 0.001,
    "log_to_console" => false,
)

@testset "RangeReserveWithDeliverabilityConstraints + AreaPTDFPowerModel (reference): separate reserves" begin
    sys = build_system(PSISystems, "two_area_pjm_DA"; add_reserves=true)
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

    components_outages_names = ["Sundance_1", "Sundance_2"]
    reserve_names = ["Reserve1_1", "Reserve1_2"]
    for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
        # --- Create Outage Data ---
        transition_data = GeometricDistributionForcedOutage(;
            mean_time_to_recovery=10,
            outage_transition_probability=1.0,
        )
        # --- Add Outage Supplemental attribute to device and services that should respond ---
        component = get_component(ThermalStandard, sys, component_name)
        add_supplemental_attribute!(sys, component, transition_data)
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
        add_supplemental_attribute!(sys, reserve_up, transition_data)
    end

    template = ProblemTemplate(NetworkModel(AreaPTDFPowerModel))
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    set_device_model!(template, DeviceModel(MonitoredLine, StaticBranch))  # Only assign Monitored Line to subsystem A template
    set_device_model!(template, Line, StaticBranch)
    set_service_model!(
        template,
        ServiceModel(
            VariableReserve{ReserveUp},
            RangeReserveWithDeliverabilityConstraints,
            "Reserve1_1",
        ),
    )
    set_service_model!(
        template,
        ServiceModel(
            VariableReserve{ReserveUp},
            RangeReserveWithDeliverabilityConstraints,
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

    flows =
        read_variable(res, "FlowActivePowerVariable__Line", table_format=TableFormat.WIDE)
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

@testset "RangeReserveWithDeliverabilityConstraints + SplitAreaPTDFPowerModel: separate reserves" begin
    sys = build_system(PSISystems, "two_area_pjm_DA"; add_reserves=true)
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

    components_outages_names = ["Sundance_1", "Sundance_2"]
    reserve_names = ["Reserve1_1", "Reserve1_2"]
    for (component_name, reserve_name) in zip(components_outages_names, reserve_names)
        # --- Create Outage Data ---
        transition_data = GeometricDistributionForcedOutage(;
            mean_time_to_recovery=10,
            outage_transition_probability=1.0,
        )
        # --- Add Outage Supplemental attribute to device and services that should respond ---
        component = get_component(ThermalStandard, sys, component_name)
        add_supplemental_attribute!(sys, component, transition_data)
        reserve_up = get_component(VariableReserve{ReserveUp}, sys, reserve_name)
        add_supplemental_attribute!(sys, reserve_up, transition_data)
    end

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
            RangeReserveWithDeliverabilityConstraints,
            "Reserve1_1",
        ),
        "a",
    )
    set_service_model!(
        template,
        ServiceModel(
            VariableReserve{ReserveUp},
            RangeReserveWithDeliverabilityConstraints,
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

    flows = get_variable_values(res)[PSI.VariableKey{FlowActivePowerVariable, Line}("")]
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
