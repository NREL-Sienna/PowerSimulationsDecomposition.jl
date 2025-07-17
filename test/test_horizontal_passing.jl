
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
    # Test values to ensure implementation changes aren't causing unexpected changes in results
    @test read_realized_variable(results_rt, "ActivePowerBalance__ACBus")[1, "116"] ==
          -0.3456209797192982
    @test read_realized_variable(results_rt, "ActivePowerBalance__ACBus")[1, "119"] ==
          -0.6255739732919298
end

@testset "Horizontal passing; compare branch models without emulator" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    results_original, _ = run_rts_multi_stage_decomposition_simulation(
        [sys, sys2];
        NT=5,
        mode="horizontal",
        monitored_line_formulations=[StaticBranchUnbounded, StaticBranchUnbounded],
        use_emulator=false,
    )
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    results_se_line, _ = run_rts_multi_stage_decomposition_simulation(
        [sys, sys2];
        NT=5,
        mode="horizontal",
        monitored_line_formulations=[
            StaticBranchUnbounded,
            StaticBranchUnboundedStateEstimation,
        ],
        use_emulator=false,
    )
    results_sub_original = get_decision_problem_results(results_original, "UC_Subsystem")
    results_sub_se_line = get_decision_problem_results(results_se_line, "UC_Subsystem")
    flow_sub_original = read_realized_variable(
        results_sub_original,
        "FlowActivePowerVariable__MonitoredLine",
    )
    flow_sub_se_line = read_realized_variable(
        results_sub_se_line,
        "FlowActivePowerVariable__MonitoredLine",
    )

    # WITHOUT the emulator, we expect some difference in the flows outside of the first timestep:
    @test isapprox(flow_sub_original[1, "A28"], flow_sub_se_line[1, "A28"])
end
