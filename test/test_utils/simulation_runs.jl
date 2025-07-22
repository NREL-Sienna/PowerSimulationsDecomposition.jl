function run_rts_multi_stage_decomposition_simulation(
    systems;
    NT=5,
    mode="vertical",
    monitored_line_formulations=[StaticBranchUnbounded, StaticBranchUnbounded],
    use_emulator=false,
)
    modeled_lines = ["CA-1", "CB-1", "AB1", "A28"]
    convert_to_monitored_line = [(name="A28", flow_limit=20.0)]
    sys = systems[1]
    sys2 = systems[2]
    transform_single_time_series!(sys, Hour(NT), Hour(NT))
    if mode == "vertical"
        transform_single_time_series!(sys2, Hour(NT), Hour(NT))
    elseif mode == "horizontal"
        transform_single_time_series!(sys2, Hour(1), Hour(1))
    end
    if use_emulator
        @assert length(systems) == 3
        sys3 = systems[3]
        transform_single_time_series!(sys3, Hour(1), Hour(1))
    end
    area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "a")

    make_subsystems!(sys2, area_subsystem_map)

    for b in modeled_lines
        l = get_component(ACBranch, sys2, b)
        PowerSystems.add_component_to_subsystem!(sys2, "a", l)
        #PowerSystems.add_component_to_subsystem!(sys2, "b", l)
    end
    for sys in systems
        add_interchanges!(sys)
        add_monitored_lines!(sys, convert_to_monitored_line)
    end

    template_uc = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks=true))
    set_device_model!(template_uc, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
    set_device_model!(template_uc, AreaInterchange, StaticBranch)
    set_device_model!(
        template_uc,
        DeviceModel(
            Line,
            StaticBranchUnbounded;
            use_slacks=true,
            attributes=Dict("filter_function" => x -> get_name(x) in modeled_lines),
        ),
    )
    set_device_model!(
        template_uc,
        DeviceModel(
            MonitoredLine,
            monitored_line_formulations[1];
            use_slacks=true,
            attributes=Dict("filter_function" => x -> get_name(x) in modeled_lines),
        ),
    )
    # Set up Model 2 (MultiProblem)
    template_uc2 = MultiProblemTemplate(
        NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true),
        ["a", "b"],
    )
    set_device_model!(template_uc2, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template_uc2, PowerLoad, StaticPowerLoad)
    set_device_model!(template_uc2, AreaInterchange, StaticBranch)
    set_device_model!(
        template_uc2,
        DeviceModel(
            Line,
            StaticBranchUnbounded;
            use_slacks=true,
            attributes=Dict("filter_function" => x -> get_name(x) in modeled_lines),
        ),
    )
    set_device_model!(
        template_uc2,
        DeviceModel(
            MonitoredLine,
            monitored_line_formulations[2];
            use_slacks=true,
            attributes=Dict("filter_function" => x -> get_name(x) in modeled_lines),
        ),
    )
    if use_emulator
        template_em = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks=true))
        set_device_model!(template_em, ThermalStandard, ThermalBasicUnitCommitment)
        set_device_model!(template_em, PowerLoad, StaticPowerLoad)
        set_device_model!(template_em, AreaInterchange, StaticBranch)
        set_device_model!(
            template_em,
            DeviceModel(
                Line,
                StaticBranchUnbounded;
                use_slacks=true,
                attributes=Dict("filter_function" => x -> get_name(x) in modeled_lines),
            ),
        )
        set_device_model!(
            template_em,
            DeviceModel(
                MonitoredLine,
                monitored_line_formulations[3];
                use_slacks=true,
                attributes=Dict("filter_function" => x -> get_name(x) in modeled_lines),
            ),
        )
    end
    models = SimulationModels(;
        decision_models=[
            DecisionModel(
                template_uc,
                sys;
                name="UC0",
                optimizer=optimizer_with_attributes(HiGHS.Optimizer),
                system_to_file=false,
                optimizer_solve_log_print=false,
                direct_mode_optimizer=true,
                store_variable_names=true,
                calculate_conflict=true,
            ),
            DecisionModel(
                MultiRegionProblem,
                template_uc2,
                sys2;
                name="UC_Subsystem",
                optimizer=optimizer_with_attributes(HiGHS.Optimizer),
                system_to_file=false,
                initialize_model=true,
                optimizer_solve_log_print=false,
                direct_mode_optimizer=true,
                rebuild_model=false,
                store_variable_names=true,
                calculate_conflict=true,
            ),
        ],
        emulation_model=use_emulator ?
                        EmulationModel(
            template_em,
            sys3;
            name="EM",
            optimizer=optimizer_with_attributes(HiGHS.Optimizer),
            store_variable_names=true,
        ) : nothing,
    )
    uc_simulation_ff = Vector{PowerSimulations.AbstractAffectFeedforward}()
    if mode == "vertical"
        FVFF_area_interchange = FixValueFeedforward(;
            component_type=AreaInterchange,
            source=FlowActivePowerVariable,
            affected_values=[FlowActivePowerVariable],
        )
        push!(uc_simulation_ff, FVFF_area_interchange)
    end
    sequence = SimulationSequence(;
        models=models,
        feedforwards=Dict("UC_Subsystem" => uc_simulation_ff),
        ini_cond_chronology=InterProblemChronology(),
    )

    # use different names for saving the solution
    sim = Simulation(;
        name="sim",
        steps=1,
        models=models,
        sequence=sequence,
        initial_time=DateTime("2020-01-01T00:00:00"),
        simulation_folder=mktempdir(),
    )

    build_out = build!(sim; console_level=Logging.Info, serialize=false)
    execute_status = execute!(sim; enable_progress_bar=true)

    return SimulationResults(sim), sim
end
