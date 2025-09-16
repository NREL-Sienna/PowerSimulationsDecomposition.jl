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
    read_realized_variable(results_uc0, "ActivePowerBalance__ACBus")
    # Tests "vertical passing": ActivePowerBalance__ACBus from the full system problem are passed as StateEstimationInjections__ACBus
    # for the MultiProblem (for the same timesteps)
    for b in [string(get_number(x)) for x in get_components(ACBus, sys)]
        apb = read_realized_variable(results_uc0, "ActivePowerBalance__ACBus")[!, b]
        sei =
            read_realized_variable(results_ucsub, "StateEstimationInjections__ACBus")[!, b]
        @test isapprox(apb, sei)
    end
    # Test values to ensure implementation changes aren't causing unexpected changes in results
    @test read_realized_variable(results_uc0, "ActivePowerBalance__ACBus")[1, "116"] ==
          -0.3456209797192982
    @test read_realized_variable(results_uc0, "ActivePowerBalance__ACBus")[1, "119"] ==
          -0.6255739732919298

    # NOTE - open issue for results processing (this is only testing a single value at t=0): https://github.com/NREL-Sienna/PowerSimulations.jl/issues/1307
    # We can manually go in and grab the results from the container to test all 5 values: 
    param = sim.models.decision_models[2].internal.container.parameters
    state_estimation_injection =
        param[InfrastructureSystems.Optimization.ParameterKey{
            PowerSimulationsDecomposition.StateEstimationInjections,
            ACBus,
        }(
            "",
        )].parameter_array
    expr = sim.models.decision_models[1].internal.container.expressions
    active_power_balance =
        expr[InfrastructureSystems.Optimization.ExpressionKey{ActivePowerBalance, ACBus}(
            "",
        )]
    for b_number in [get_number(x) for x in get_components(ACBus, sys)]
        apb = value.(active_power_balance[b_number, :]).data
        sei = state_estimation_injection[string(b_number), :].data
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
        "FlowActivePowerVariable__MonitoredLine",
    )
    flow_sub_se_line = read_realized_variable(
        results_sub_se_line,
        "FlowActivePowerVariable__MonitoredLine",
    )

    @test isapprox(flow_sub_original[1, "A28"], flow_sub_se_line[1, "A28"])

    # NOTE - open issue for results processing (this is only testing a single value at t=0): https://github.com/NREL-Sienna/PowerSimulations.jl/issues/1307
    # We can manually go in and grab the results from the container to test all 5 values: 
    vars_original = sim_original.models.decision_models[2].internal.container.variables
    flows_original = Vector(
        vars_original[PowerSimulations.VariableKey{FlowActivePowerVariable, MonitoredLine}(
            "",
        )][
            "A28",
            :,
        ],
    )
    vars_se_line = sim_se_line.models.decision_models[2].internal.container.variables
    flows_se_line = Vector(
        vars_se_line[PowerSimulations.VariableKey{FlowActivePowerVariable, MonitoredLine}(
            "",
        )][
            "A28",
            :,
        ],
    )
    @test isapprox(flows_original, flows_se_line)

