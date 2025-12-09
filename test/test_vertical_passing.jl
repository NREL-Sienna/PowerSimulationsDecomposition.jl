@testset "Test vertical passing without emulator" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    results, sim = run_rts_multi_stage_decomposition_simulation(
        [sys, sys2];
        NT=5,
        mode="vertical",
        monitored_line_formulations=[StaticBranchUnbounded, StaticBranchUnbounded],
        use_emulator=false,
    )
    results_uc0 = get_decision_problem_results(results, "UC0")
    results_ucsub = get_decision_problem_results(results, "UC_Subsystem")
    results_em = get_emulation_problem_results(results)
    read_realized_variable(
        results_uc0,
        "ActivePowerBalance__ACBus";
        table_format=TableFormat.WIDE,
    )
    # Tests "vertical passing": ActivePowerBalance__ACBus from the full system problem are passed as StateEstimationInjections__ACBus
    # for the MultiProblem (for the same timesteps)
    for b in [string(get_number(x)) for x in get_components(ACBus, sys)]
        apb = read_realized_variable(
            results_uc0,
            "ActivePowerBalance__ACBus";
            table_format=TableFormat.WIDE,
        )[
            !,
            b,
        ]
        sei = read_realized_variable(
            results_ucsub,
            "StateEstimationInjections__ACBus";
            table_format=TableFormat.WIDE,
        )[
            !,
            b,
        ]
        @test isapprox(apb, sei)
    end
    # Test values to ensure implementation changes aren't causing unexpected changes in results
    @test read_realized_variable(
        results_uc0,
        "ActivePowerBalance__ACBus";
        table_format=TableFormat.WIDE,
    )[
        1,
        "116",
    ] == -0.3456209797192982
    @test read_realized_variable(
        results_uc0,
        "ActivePowerBalance__ACBus";
        table_format=TableFormat.WIDE,
    )[
        1,
        "119",
    ] == -0.6255739732919298

    state_estimation_injection = read_realized_variable(
        results_ucsub,
        "StateEstimationInjections__ACBus";
        table_format=TableFormat.WIDE,
    )
    active_power_balance = read_realized_variable(
        results_uc0,
        "ActivePowerBalance__ACBus";
        table_format=TableFormat.WIDE,
    )

    for b_number in [get_number(x) for x in get_components(ACBus, sys)]
        apb = value.(active_power_balance[:, string(b_number)])
        sei = state_estimation_injection[:, string(b_number)]
        @test isapprox(apb, sei)
    end
end

@testset "Vertical passing; compare branch models without emulator" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    results_original, sim_original = run_rts_multi_stage_decomposition_simulation(
        [sys, sys2];
        NT=5,
        mode="vertical",
        monitored_line_formulations=[StaticBranchUnbounded, StaticBranchUnbounded],
        use_emulator=false,
    )
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    results_se_line, sim_se_line = run_rts_multi_stage_decomposition_simulation(
        [sys, sys2];
        NT=5,
        mode="vertical",
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
        "PTDFBranchFlow__MonitoredLine";
        table_format=TableFormat.WIDE,
    )
    flow_sub_se_line = read_realized_variable(
        results_sub_se_line,
        "PTDFBranchFlow__MonitoredLine";
        table_format=TableFormat.WIDE,
    )

    @test isapprox(flow_sub_original[:, "A28"], flow_sub_se_line[:, "A28"])
end
