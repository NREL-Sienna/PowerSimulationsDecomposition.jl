
@testset "Test horizontal passing without emulator" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    results, _ = run_rts_multi_stage_decomposition_simulation(
        [sys, sys2];
        NT=5,
        mode="horizontal",
        monitored_line_formulations=[StaticBranchUnbounded, StaticBranchUnbounded],
        use_emulator=false,
    )
    results_uc = get_decision_problem_results(results, "UC0")
    results_rt = get_decision_problem_results(results, "UC_Subsystem")

    #Test "horizontal passing": ActivePowerBalance__ACBus(t-1) =  StateEstimationInjections__ACBus(t).
    #For the real time problem, the state estimation comes from the previous time interval. 
    for b in [string(get_number(x)) for x in get_components(ACBus, sys)]
        apb = read_realized_variable(results_rt, "ActivePowerBalance__ACBus")[!, b]
        sei = read_realized_variable(results_rt, "StateEstimationInjections__ACBus")[!, b]
        @test isapprox(sei[2:end], apb[1:(end - 1)])
    end
end
