@testset "Coordination algorithms build and execute" begin
    new_branch_ratings = Dict(
        "A11" => 1.5,
        "A28" => 1000.0,
        "B28" => 1000.0,
        "CA-1" => 1000.0,
        "CB-1" => 1000.0,
        "AB1" => 1000.0,
        "AB2" => 1000.0,
        "AB3" => 1000.0,
        "A18" => 1000.0,
        "A20" => 1000.0,
        "A22" => 1000.0,
    )
    modeled_monitored_lines = ["A11"]

    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")

    hvdc_bus_area_correction!(sys, x -> x.arc.to.area)
    add_all_unbounded_interchanges!(sys)
    set_new_branch_ratings!(sys, new_branch_ratings)
    convert_to_monitored_line!(sys, modeled_monitored_lines)

    @test run_rts_coordination_simulation(sys, Dict()) == IS.Simulation.RunStatusModule.RunStatus.SUCCESSFULLY_FINALIZED

    @test run_rts_coordination_simulation(
        sys,
        Dict(3 => CostCurveCoordination(
            coordinated_lines = ["A11"],
            coordinated_regions = ["a", "b"],
            num_segments = 101,
            check_range_rate = 0.5,
        )),
    ) == IS.Simulation.RunStatusModule.RunStatus.SUCCESSFULLY_FINALIZED

    @test run_rts_coordination_simulation(
        sys,
        Dict(3 => ShadowPriceCoordination(
            coordinated_lines = ["A11"],
            coordinated_regions = ["a", "b"],
        )),
    ) == IS.Simulation.RunStatusModule.RunStatus.SUCCESSFULLY_FINALIZED
end
