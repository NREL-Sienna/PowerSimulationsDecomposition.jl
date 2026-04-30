@testset "Test branches are assigned to subsystems correctly" begin
    sys = build_system(PSISystems, "two_area_pjm_DA"; add_reserves=true)

    area_subsystem_map = Dict("Area1" => "a", "Area2" => "b")
    make_subsystems!(sys, area_subsystem_map)
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
    set_device_model!(template, DeviceModel(Line, StaticBranch))

    problem = DecisionModel(
        MultiRegionProblem,
        template,
        sys;
        horizon = Hour(24),
        interval = Hour(1),
        resolution = Hour(1),
        name="UC_Subsystem",
        optimizer=HiGHS_optimizer_small_gap,
    )

    build_out = build!(problem; output_dir=mktempdir())

    @test size(
        problem.internal.container.subproblems["a"].constraints[PSI.ConstraintKey{
            FlowRateConstraint,
            Line,
        }(
            "lb",
        )],
    )[1] == 6
    @test size(
        problem.internal.container.subproblems["b"].constraints[PSI.ConstraintKey{
            FlowRateConstraint,
            Line,
        }(
            "lb",
        )],
    )[1] == 6
    @test size(
        problem.internal.container.subproblems["a"].constraints[PSI.ConstraintKey{
            FlowRateConstraint,
            MonitoredLine,
        }(
            "lb",
        )],
    )[1] == 1
    @test !haskey(
        problem.internal.container.subproblems["b"].constraints,
        PSI.ConstraintKey{FlowRateConstraint, MonitoredLine}("lb"),
    )
end
