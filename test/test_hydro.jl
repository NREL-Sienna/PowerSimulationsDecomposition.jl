@testset "Test decomposition with hydro budget (1D results)" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    add_hydro_budget_time_series_to_rts!(sys)
    sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    add_hydro_budget_time_series_to_rts!(sys2)
    results, _ = run_rts_multi_stage_decomposition_simulation(
        [sys, sys2];
        NT=5,
        mode="horizontal",
        monitored_line_formulations=[StaticBranchUnbounded, StaticBranchUnbounded],
        use_emulator=false,
        add_reserves=true,
        add_hydro=true,
    )
    @test isa(results, SimulationResults)
end
