function run_rts_multi_stage_decomposition_simulation(
    sys;
    NT=5,
    mode="vertical",
    monitored_line_formulations=[StaticBranchUnbounded, StaticBranchUnbounded],
    use_emulator=false,
    add_reserves=false,
    add_hydro=false,
    in_memory=false,
)
    modeled_lines = ["CA-1", "CB-1", "AB1", "A28"]
    convert_to_monitored_line = [(name="A28", flow_limit=20.0)]
    area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "a")

    make_subsystems!(sys, area_subsystem_map)

    for b in modeled_lines
        l = get_component(ACBranch, sys, b)
        to_bus_area = get_name(get_area(get_to(get_arc(l))))
        PowerSystems.add_component_to_subsystem!(sys, area_subsystem_map[to_bus_area], l)
        @info "Assigning line $(get_name(l)) to subsystem $(area_subsystem_map[to_bus_area])"
    end
    #Add all area interchanges and include in all subsystems
    add_interchanges!(sys)
    for b in get_components(AreaInterchange, sys)
        for subsystem in unique(values(area_subsystem_map))
            add_component_to_subsystem!(sys, subsystem, b)
        end
    end
    add_monitored_lines!(sys, convert_to_monitored_line)

    template_uc = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks=true))
    set_device_model!(template_uc, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
    add_hydro &&
        set_device_model!(template_uc, HydroDispatch, HydroDispatchRunOfRiverBudget)
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
    if add_reserves
        # add reserve formulations:
        set_service_model!(
            template_uc,
            ServiceModel(
                VariableReserve{ReserveUp},
                RampReserveWithDeliverabilityConstraints,
                "Spin_Up_R1",
            ),
        )
        set_service_model!(
            template_uc,
            ServiceModel(
                VariableReserve{ReserveUp},
                RampReserveWithDeliverabilityConstraints,
                "Spin_Up_R2",
            ),
        )
        set_service_model!(
            template_uc,
            ServiceModel(
                VariableReserve{ReserveUp},
                RampReserveWithDeliverabilityConstraints,
                "Spin_Up_R3",
            ),
        )
    end
    # Set up Model 2 (MultiProblem)
    template_uc2 = MultiProblemTemplate(
        NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true),
        ["a", "b"],
    )
    set_device_model!(template_uc2, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template_uc2, PowerLoad, StaticPowerLoad)
    add_hydro &&
        set_device_model!(template_uc2, HydroDispatch, HydroDispatchRunOfRiverBudget)
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
    if add_reserves
        # add modeled reserve components to subsystems: 
        r1 = get_component(VariableReserve{ReserveUp}, sys, "Spin_Up_R1")
        add_component_to_subsystem!(sys, "a", r1)
        r2 = get_component(VariableReserve{ReserveUp}, sys, "Spin_Up_R2")
        add_component_to_subsystem!(sys, "b", r2)
        r3 = get_component(VariableReserve{ReserveUp}, sys, "Spin_Up_R3")
        add_component_to_subsystem!(sys, "a", r3)

        # add reserve formulations (also to specific subsystems):
        set_service_model!(
            template_uc2,
            ServiceModel(
                VariableReserve{ReserveUp},
                RampReserveWithDeliverabilityConstraints,
                "Spin_Up_R1",
            ),
            "a",
        )
        set_service_model!(
            template_uc2,
            ServiceModel(
                VariableReserve{ReserveUp},
                RampReserveWithDeliverabilityConstraints,
                "Spin_Up_R2",
            ),
            "b",
        )
        set_service_model!(
            template_uc2,
            ServiceModel(
                VariableReserve{ReserveUp},
                RampReserveWithDeliverabilityConstraints,
                "Spin_Up_R3",
            ),
            "a",
        )
    end
    if use_emulator
        template_em = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks=true))
        set_device_model!(template_em, ThermalStandard, ThermalBasicUnitCommitment)
        set_device_model!(template_em, PowerLoad, StaticPowerLoad)
        add_hydro &&
            set_device_model!(template_em, HydroDispatch, HydroDispatchRunOfRiverBudget)
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
    if mode == "vertical"
        d2_horizon = Hour(NT)
        d2_interval = Hour(NT)
    elseif mode == "horizontal"
        d2_horizon = Hour(1)
        d2_interval = Hour(1)
    end
    models = SimulationModels(;
        decision_models=[
            DecisionModel(
                template_uc,
                sys;
                name="UC0",
                optimizer=optimizer_with_attributes(HiGHS.Optimizer),
                horizon=Hour(NT),
                interval=Hour(NT),
                resolution=Hour(1),
                optimizer_solve_log_print=false,
                direct_mode_optimizer=true,
                store_variable_names=true,
                calculate_conflict=true,
            ),
            DecisionModel(
                MultiRegionProblem,
                template_uc2,
                sys;
                name="UC_Subsystem",
                optimizer=optimizer_with_attributes(HiGHS.Optimizer),
                horizon=d2_horizon,
                interval=d2_interval,
                resolution=Hour(1),
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
            sys;
            name="EM",
            optimizer=optimizer_with_attributes(HiGHS.Optimizer),
            resolution=Hour(1),
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

    build_out = build!(sim; console_level=Logging.Info)
    execute_status = execute!(sim; in_memory=in_memory, enable_progress_bar=true)

    return SimulationResults(sim), sim
end
