NT = 5
modeled_lines = ["CA-1"]
convert_to_monitored_line = [(name="A28", flow_limit=20)]

sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
transform_single_time_series!(sys, Hour(NT), Hour(NT))
transform_single_time_series!(sys2, Hour(NT), Hour(NT))

area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "b")

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

hvdc = get_component(TwoTerminalHVDCLine, sys2, "DC1")
PowerSystems.add_component_to_subsystem!(sys2, "a", hvdc)          
PowerSystems.add_component_to_subsystem!(sys2, "b", hvdc)

# Set up Model 1 
template_uc = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks=true))
set_device_model!(template_uc, ThermalStandard, ThermalBasicUnitCommitment)
set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
set_device_model!(template_uc, AreaInterchange, StaticBranch)
set_device_model!(template_uc, TwoTerminalHVDCLine, HVDCTwoTerminalLossless)
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
# Set up Model 2 (MultiProblem)
template_uc2 =
    MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true), ["a", "b"])
set_device_model!(template_uc2, ThermalStandard, ThermalBasicUnitCommitment)
set_device_model!(template_uc2, PowerLoad, StaticPowerLoad)
set_device_model!(template_uc2, AreaInterchange, StaticBranch)
set_device_model!(template_uc2, TwoTerminalHVDCLine, HVDCTwoTerminalLossless)
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
            optimizer_solve_log_print=true,
            direct_mode_optimizer=true,
            rebuild_model=false,
            store_variable_names=true,
            calculate_conflict=true,
        ),
    ],
)

uc_simulation_ff = Vector{PowerSimulations.AbstractAffectFeedforward}()
FVFF_area_interchange = FixValueFeedforward(;
    component_type=AreaInterchange,
    source=FlowActivePowerVariable,
    affected_values=[FlowActivePowerVariable],
)
push!(uc_simulation_ff, FVFF_area_interchange)

sequence = SimulationSequence(;
    models=models,
    feedforwards=Dict("UC_Subsystem" => uc_simulation_ff),
    ini_cond_chronology=InterProblemChronology(),
);

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
results_uc0 = get_decision_problem_results(results, "UC0")
results_ucsub = get_decision_problem_results(results, "UC_Subsystem")


# Tests for double counting bug of HVDC
hvdc_in_a = length(get_components(TwoTerminalHVDCLine, sys2, subsystem_name = "a")) > 0
hvdc_in_b = length(get_components(TwoTerminalHVDCLine, sys2, subsystem_name = "b")) > 0
if hvdc_in_a
    apb_b_boundary_bus =  read_realized_variable(results_uc0, "ActivePowerBalance__ACBus")[1, "316"]
    sei_b_boundary_bus =  read_realized_variable(results_ucsub, "StateEstimationInjections__ACBus")[1, "316"]
    hvdc_a_to_b = read_realized_variable(results_ucsub, "FlowActivePowerVariable__TwoTerminalHVDCLine")[1, "DC1"]/100
    @test isapprox(apb_b_boundary_bus, sei_b_boundary_bus + hvdc_a_to_b)
end 
if hvdc_in_b 
    apb_a_boundary_bus =  read_realized_variable(results_uc0, "ActivePowerBalance__ACBus")[1, "113"]
    sei_a_boundary_bus =  read_realized_variable(results_ucsub, "StateEstimationInjections__ACBus")[1, "113"]
    hvdc_a_to_b = read_realized_variable(results_ucsub, "FlowActivePowerVariable__TwoTerminalHVDCLine")[1, "DC1"]/100
    @test isapprox(apb_a_boundary_bus, sei_a_boundary_bus - hvdc_a_to_b)
end 
