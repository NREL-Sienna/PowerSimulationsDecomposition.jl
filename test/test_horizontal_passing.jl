NT = 24
modeled_lines = ["CA-1", "CB-1", "AB1", "A28"]
convert_to_monitored_line = [(name="A28", flow_limit=0.2)]

sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
transform_single_time_series!(sys, Hour(NT), Hour(NT))
transform_single_time_series!(sys2, Hour(1), Hour(1))

area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "a")

make_subsystems!(sys2, area_subsystem_map)

for b in modeled_lines
    l = get_component(ACBranch, sys2, b)
    PowerSystems.add_component_to_subsystem!(sys2, "a", l)
    #PowerSystems.add_component_to_subsystem!(sys2, "b", l)
end

add_interchanges!(sys)
add_interchanges!(sys2)

add_monitored_lines!(sys, convert_to_monitored_line)
add_monitored_lines!(sys2, convert_to_monitored_line)

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
        StaticBranchUnbounded;
        use_slacks=true,
        attributes=Dict("filter_function" => x -> get_name(x) in modeled_lines),
    ),
)

template_uc2 =
    MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true), ["a", "b"])
set_device_model!(template_uc2, AreaInterchange, StaticBranch)
set_device_model!(template_uc2, ThermalStandard, ThermalBasicUnitCommitment)
set_device_model!(template_uc2, PowerLoad, StaticPowerLoad)
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
        StaticBranchUnbounded;
        use_slacks=true,
        attributes=Dict("filter_function" => x -> get_name(x) in modeled_lines),
    ),
)

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
)
uc_simulation_ff = Vector{PowerSimulations.AbstractAffectFeedforward}()
sequence = SimulationSequence(;
    models=models,
    feedforwards=Dict("UC_Subsystem" => uc_simulation_ff),
    ini_cond_chronology=InterProblemChronology(),
);

# use different names for saving the solution
sim = Simulation(;
    name="sim",
    steps=1,
    models=models,
    sequence=sequence,
    initial_time=DateTime("2020-01-01T00:00:00"),
    simulation_folder=mktempdir(),
);

build_out = build!(sim; console_level=Logging.Info, serialize=false)
execute_status = execute!(sim; enable_progress_bar=true);

results = SimulationResults(sim)
results_uc = get_decision_problem_results(results, "UC0")
results_rt = get_decision_problem_results(results, "UC_Subsystem")

#Test "horizontal passing": ActivePowerBalance__ACBus(t-1) =  StateEstimationInjections__ACBus(t).
#For the real time problem, the state estimation comes from the previous time interval. 
for b in [string(get_number(x)) for x in get_components(ACBus, sys)]
    apb = read_realized_variable(results_rt, "ActivePowerBalance__ACBus")[!, b]
    sei = read_realized_variable(results_rt, "StateEstimationInjections__ACBus")[!, b]
    @test isapprox(sei[2:end], apb[1:(end - 1)])
end

#TODO - write additional tests based on ychen's testing scripts. 
