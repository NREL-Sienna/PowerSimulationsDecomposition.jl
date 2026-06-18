using Pkg
Pkg.activate("../test")
# ] add PowerSimulations#rh/area_ptdf_fixes

# SIIP Packages
using Revise
using PowerSimulationsDecomposition
using PowerSimulations
using PowerSystems
using PowerSystemCaseBuilder
using InfrastructureSystems
using PowerNetworkMatrices
using HydroPowerSimulations
import PowerSystemCaseBuilder: PSITestSystems
using PowerNetworkMatrices
using StorageSystemsSimulations
using TimeSeries
using Dates
using HiGHS
using JuMP
using Xpress
const IS = InfrastructureSystems
const PSI = PowerSimulations
const PSY = PowerSystems

# Test Packages
using Test
using Logging

include("test_utils/system_modifications.jl")
include("test_utils/model_checks.jl")
include("test_utils/simulation_runs.jl")

da_sys_name = "modified_RTS_GMLC_DA_sys_noForecast_with_offshore_hvdc_ptdf.json"
sys = System(da_sys_name; time_series_read_only=false, runchecks=false)

#buses_no_areas = get_components(x -> isnothing(x.area), ACBus, sys)
buses_no_areas = get_components(x -> isnothing(x.area), ACBus, sys)
offshore_area = get_component(Area, sys, "Offshore")
for bus in buses_no_areas
    set_area!(bus, offshore_area)
    #set_available!(bus, false)
end
for dcbus in get_components(PSY.DCBus, sys)
    set_area!(dcbus, offshore_area)
    #subsystem_str = "a"
    #dcbus.ext["subsystem"] = subsystem_str
end

ic_buses = ["Anna", "Bardeen", "Cary"]
for bus_name in ic_buses
    bus = get_component(ACBus, sys, bus_name)
    set_area!(bus, offshore_area)
end

island_buses = ["401_DC_ACBus", "401_DC_ACBus_REF"]
island_area = Area("Island")
add_component!(sys, island_area)
for bus_name in island_buses
    bus = get_component(ACBus, sys, bus_name)
    set_area!(bus, island_area)
end
#set_available!(get_component(RenewableDispatch, sys, "wind-ofs-401"), false)
#remove_component!(sys, offshore_area)

#@testset "Test vertical passing without emulator" begin
#    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
results, sim = run_rts_multi_stage_decomposition_simulation(
    sys;
    NT=5,
    mode="vertical",
    monitored_line_formulations=[StaticBranchUnbounded, StaticBranchUnbounded],
    use_emulator=false,
    add_hvdc=true,
);

cons_base_problem =
    sim.models.decision_models[1].internal.container.constraints[InfrastructureSystems.Optimization.ConstraintKey{
        NodalBalanceActiveConstraint,
        DCBus,
    }(
        "",
    )]
cons_a =
    sim.models.decision_models[2].internal.container.subproblems["a"].constraints[InfrastructureSystems.Optimization.ConstraintKey{
        NodalBalanceActiveConstraint,
        DCBus,
    }(
        "",
    )]

uc1=sim.models.decision_models[1].internal.container
uc2a=sim.models.decision_models[2].internal.container.subproblems["a"]
uc2b=sim.models.decision_models[2].internal.container.subproblems["b"]
uc2c=sim.models.decision_models[2].internal.container.subproblems["c"]

uc2a.constraints
uc2a.constraints[InfrastructureSystems.Optimization.ConstraintKey{CopperPlateBalanceConstraint, Area}("")]
uc2a.constraints[InfrastructureSystems.Optimization.ConstraintKey{NodalBalanceActiveConstraint, DCBus}("")]

##########################################################################################

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

@testset "Vertical passing; compare branch models without emulator" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    results_original, sim_original = run_rts_multi_stage_decomposition_simulation(
        sys;
        NT=5,
        mode="vertical",
        monitored_line_formulations=[StaticBranchUnbounded, StaticBranchUnbounded],
        use_emulator=false,
    )
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    results_se_line, sim_se_line = run_rts_multi_stage_decomposition_simulation(
        sys;
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
