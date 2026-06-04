struct SubsytemAssignmentData
    area_subsystem_map::Dict{String, String}
    branch_assignment_function::Union{Nothing, Function}
    interchange_lines::Vector{String}
end

function SubsytemAssignmentData(;
    area_subsystem_map = Dict{String, String}(),
    branch_assignment_function = nothing,
    interchange_lines = String[],
)
    return SubsytemAssignmentData(
        area_subsystem_map,
        branch_assignment_function,
        interchange_lines,
    )
end

function remove_service_contributing_devices!(sys, bus)
    for static_injection in get_components(x -> x.bus == bus, StaticInjection, sys)
        for service in collect(get_services(static_injection))
            remove_service!(static_injection, service)
        end
    end
end

function hvdc_bus_area_correction!(sys, assignment_function)
    for hvdc in get_available_components(TwoTerminalHVDC, sys)
        from_bus = hvdc.arc.from
        to_bus = hvdc.arc.to
        from_area = from_bus.area
        to_area = to_bus.area
        if from_area != to_area
            assigned_area = assignment_function(hvdc)
            if assigned_area == from_area
                remove_service_contributing_devices!(sys, to_bus)
                set_area!(to_bus, assigned_area)
            elseif assigned_area == to_area
                remove_service_contributing_devices!(sys, from_bus)
                set_area!(from_bus, assigned_area)
            else
                error("HVDC bus area assignment function returned an area different from both terminals")
            end
        end
    end
end

function add_all_unbounded_interchanges!(sys)
    areas = collect(get_components(Area, sys))
    area_names = [get_name(x) for x in areas]
    areas_sorted = areas[sortperm(area_names)]
    for i in areas_sorted
        for j in areas_sorted
            i_name = get_name(i)
            j_name = get_name(j)
            if i_name <= j_name
                continue
            end
            interchange = AreaInterchange(;
                name = i_name * "_" * j_name,
                available = true,
                active_power_flow = 0.0,
                from_area = i,
                to_area = j,
                flow_limits = (from_to = 999999, to_from = 999999),
            )
            add_component!(sys, interchange)
        end
    end
end

function set_new_branch_ratings!(sys, branch_rating_dict)
    for (branch_name, new_rating) in branch_rating_dict
        l = get_component(ACBranch, sys, branch_name)
        if !isnothing(l)
            set_rating!(l, new_rating)
        end
    end
end

function convert_to_monitored_line!(sys, line_names_to_convert)
    for line_name in line_names_to_convert
        l = get_component(Line, sys, line_name)
        if !isnothing(l)
            convert_component!(sys, l, MonitoredLine)
        end
    end
end

function get_year_from_load_ts(sys::System)
    load = first(get_components(StaticLoad, sys))
    ts = get_time_series_array(SingleTimeSeries, load, "max_active_power")
    tstamps = timestamp(ts)
    return unique(Dates.year.(tstamps))[1]
end

function _assign_subsystems!(sys, subsystem_assignment_data)
    area_subsystem_map = subsystem_assignment_data.area_subsystem_map
    branch_assignment_function = subsystem_assignment_data.branch_assignment_function
    subsystems = unique([v for (_, v) in area_subsystem_map])
    for subsystem in subsystems
        add_subsystem!(sys, subsystem)
    end
    for b in get_components(Area, sys)
        add_component_to_subsystem!(sys, area_subsystem_map[get_name(b)], b)
    end
    for b in get_components(Bus, sys)
        add_component_to_subsystem!(sys, area_subsystem_map[get_name(get_area(b))], b)
    end
    for b in get_components(StaticInjection, sys)
        add_component_to_subsystem!(sys, area_subsystem_map[get_name(get_area(get_bus(b)))], b)
    end
    for b in get_components(Branch, sys)
        assigned = branch_assignment_function(b, area_subsystem_map)
        if !isnothing(assigned)
            for subsystem in assigned
                PowerSystems.add_component_to_subsystem!(sys, subsystem, b)
            end
        end
    end
end

function _add_outages!(systems_decisions, system_emulator, outage_vector)
    return
end

function make_branch_assignment_function_coord(;
    coordinated_lines_subsystem_map::Dict{String, Vector{String}},
    modeled_lines::Vector{String},
    modeled_monitored_lines::Vector{String},
)
    return function (branch, area_map)
        name = get_name(branch)
        if isa(branch, MonitoredLine) && haskey(coordinated_lines_subsystem_map, name)
            return coordinated_lines_subsystem_map[name]
        elseif isa(branch, MonitoredLine) && name in modeled_monitored_lines
            to_area = get_name(get_area(get_to(get_arc(branch))))
            return [area_map[to_area]]
        elseif isa(branch, Line) && name in modeled_lines
            return unique(values(area_map))
        elseif isa(branch, AreaInterchange)
            return unique(values(area_map))
        elseif isa(branch, TwoTerminalHVDC)
            to_area = get_name(get_area(get_to(get_arc(branch))))
            return [area_map[to_area]]
        else
            return nothing
        end
    end
end

function build_multi_stage_simulation_coord(
    sys::System,
    decision_templates_in::Vector,
    emulator_template_in::Union{Nothing, ProblemTemplate},
    timing::Vector,
    optimizer;
    sim_name = "sim",
    sim_steps = 1,
    subsystem_assignment_data::Vector{SubsytemAssignmentData} = SubsytemAssignmentData[],
    feedforwards::Dict{String, Vector{<:PowerSimulations.AbstractAffectFeedforward}} =
        Dict{String, Vector{<:PowerSimulations.AbstractAffectFeedforward}}(),
    coordination_per_stage::Dict = Dict{Int, Any}(),
    initial_time = nothing,
    simulation_folder = ".",
)
    decision_templates = [deepcopy(x) for x in decision_templates_in]
    emulator_template = deepcopy(emulator_template_in)

    systems_decision = System[]
    for ix in 1:length(decision_templates)
        s = deepcopy(sys)
        transform_single_time_series!(s, timing[ix].horizon, timing[ix].interval)
        push!(systems_decision, deepcopy(s))
    end

    system_emulator = nothing
    if !isnothing(emulator_template)
        system_emulator = deepcopy(sys)
        transform_single_time_series!(
            system_emulator, timing[end].horizon, timing[end].interval,
        )
    end

    sa_ix = 1
    for (template, sysd) in zip(decision_templates, systems_decision)
        if isa(template, MultiProblemTemplate)
            _assign_subsystems!(sysd, subsystem_assignment_data[sa_ix])
            sa_ix += 1
        end
    end

    decision_models = DecisionModel[]
    for (ix, (template, sysd)) in enumerate(zip(decision_templates, systems_decision))
        if isa(template, MultiProblemTemplate)
            coord = get(coordination_per_stage, ix, NoCoordination())
            d = DecisionModel(
                MultiRegionProblem,
                template,
                sysd;
                name = "D$(ix)",
                optimizer = optimizer,
                optimizer_solve_log_print = false,
                direct_mode_optimizer = true,
                store_variable_names = true,
                calculate_conflict = true,
                initialize_model = false,
                check_numerical_bounds = false,
                rebuild_model = false,
                coordination = coord,
            )
        else
            d = DecisionModel(
                template,
                sysd;
                name = "D$(ix)",
                optimizer = optimizer,
                optimizer_solve_log_print = false,
                direct_mode_optimizer = true,
                store_variable_names = true,
                calculate_conflict = true,
                initialize_model = false,
                check_numerical_bounds = false,
                rebuild_model = false,
            )
        end
        push!(decision_models, d)
    end

    emulation_model = if isnothing(emulator_template)
        nothing
    else
        EmulationModel(
            emulator_template,
            system_emulator;
            name = "EM",
            optimizer = optimizer,
            optimizer_solve_log_print = false,
            direct_mode_optimizer = true,
            store_variable_names = true,
            calculate_conflict = true,
        )
    end

    models = SimulationModels(;
        decision_models = decision_models,
        emulation_model = emulation_model,
    )
    sequence = SimulationSequence(;
        models = models,
        feedforwards = feedforwards,
        ini_cond_chronology = InterProblemChronology(),
    )

    sim = Simulation(;
        name = sim_name,
        steps = sim_steps,
        models = models,
        sequence = sequence,
        initial_time = initial_time,
        simulation_folder = simulation_folder,
    )
    build!(sim; console_level = Logging.Info)
    return sim
end

function run_rts_coordination_simulation(sys, coordination_per_stage)
    modeled_monitored_lines = ["A11"]
    modeled_lines = ["CA-1", "CB-1", "AB1", "A28"]
    coordinated_line_subsystems = Dict("A11" => ["a", "b"])
    area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "c")

    branch_assignment_function_coord = make_branch_assignment_function_coord(;
        coordinated_lines_subsystem_map = coordinated_line_subsystems,
        modeled_lines = modeled_lines,
        modeled_monitored_lines = modeled_monitored_lines,
    )

    template_full = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks = true))
    template_sub = MultiProblemTemplate(
        NetworkModel(SplitAreaPTDFPowerModel; use_slacks = true),
        ["a", "b", "c"],
    )
    template_em = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks = true))

    for template in [template_full, template_sub, template_em]
        set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
        set_device_model!(template, PowerLoad, StaticPowerLoad)
        set_device_model!(template, DeviceModel(AreaInterchange, StaticBranchUnbounded))
        set_device_model!(
            template,
            DeviceModel(
                Line,
                StaticBranchUnbounded;
                attributes = Dict("filter_function" => x -> get_name(x) in modeled_lines),
            ),
        )
        set_device_model!(template, TwoTerminalGenericHVDCLine, HVDCTwoTerminalLossless)
    end

    set_device_model!(template_full, DeviceModel(MonitoredLine, StaticBranchUnbounded, use_slacks = true))
    set_device_model!(template_sub, DeviceModel(MonitoredLine, StaticBranchBounds, use_slacks = true))
    set_device_model!(template_em, DeviceModel(MonitoredLine, StaticBranchBounds, use_slacks = true))

    timing_em = [
        (horizon = Hour(24), interval = Hour(24)),
        (horizon = Hour(24), interval = Hour(24)),
        (horizon = Hour(1), interval = Hour(1)),
        (horizon = Hour(1), interval = Hour(1)),
    ]

    fixed_interchange_flow = FixValueFeedforward(;
        component_type = AreaInterchange,
        source = FlowActivePowerVariable,
        affected_values = [FlowActivePowerVariable],
    )
    fixed_hvdc_flow = FixValueFeedforward(;
        component_type = TwoTerminalGenericHVDCLine,
        source = FlowActivePowerVariable,
        affected_values = [FlowActivePowerVariable],
    )
    fixed_thermal_on = FixValueFeedforward(;
        component_type = ThermalStandard,
        source = OnVariable,
        affected_values = [OnVariable],
    )
    fixed_thermal_dispatch = FixValueFeedforward(;
        component_type = ThermalStandard,
        source = ActivePowerVariable,
        affected_values = [ActivePowerVariable],
    )
    feedforwards = Dict{String, Vector{<:PowerSimulations.AbstractAffectFeedforward}}(
        "D2" => PowerSimulations.AbstractAffectFeedforward[
            fixed_interchange_flow,
            fixed_hvdc_flow,
        ],
        "D3" => PowerSimulations.AbstractAffectFeedforward[
            fixed_interchange_flow,
            fixed_hvdc_flow,
            fixed_thermal_on,
        ],
        "EM" => PowerSimulations.AbstractAffectFeedforward[
            fixed_hvdc_flow,
            fixed_thermal_on,
            fixed_thermal_dispatch,
        ],
    )

    year = get_year_from_load_ts(sys)
    ini_day = DateTime("$year-01-01T00:00:00") + Day(90)

    sim = build_multi_stage_simulation_coord(
        sys,
        [template_full, template_sub, template_sub],
        template_em,
        timing_em,
        HiGHS_optimizer_small_gap;
        sim_name = "rts_test_coord",
        sim_steps = 1,
        subsystem_assignment_data = [
            SubsytemAssignmentData(
                area_subsystem_map = area_subsystem_map,
                branch_assignment_function = branch_assignment_function_coord,
            ),
            SubsytemAssignmentData(
                area_subsystem_map = area_subsystem_map,
                branch_assignment_function = branch_assignment_function_coord,
            ),
        ],
        feedforwards = feedforwards,
        coordination_per_stage = coordination_per_stage,
        initial_time = ini_day,
        simulation_folder = mktempdir(),
    )
    return execute!(sim)
end