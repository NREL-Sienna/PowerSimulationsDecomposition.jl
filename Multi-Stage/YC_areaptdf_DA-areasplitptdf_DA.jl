using Pkg
Pkg.activate(".")
using Revise

ENV["XPRESSDIR"] = "C:\\xpressmp"
ENV["XPAUTH_PATH"] = "C:\\xpressmp\\bin"

using HydroPowerSimulations
using PowerSimulations
using PowerSystemCaseBuilder
using InfrastructureSystems
using StorageSystemsSimulations
using PowerSimulationsDecomposition
using PowerSystems
# using PowerGraphics

using Xpress
using JuMP
using Logging
using Dates
using DataFrames

#include("coordination_EnergyOnly.jl")
#include("GlobalM2M.jl")
#import StatsPlots
#import Plots
include("YC_test_function.jl")
#using Plots
#gr()
const PSY = PowerSystems
const IF = InfrastructureSystems
const PSB = PowerSystemCaseBuilder

NT=5
selected_line=["CA-1", "CB-1", "AB1"]
monitoredlined_line=["A28"]
limit=20
use_monitoredline=1

sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
transform_single_time_series!(sys, Hour(NT), Hour(NT))
transform_single_time_series!(sys2, Hour(NT), Hour(NT))
#transform_single_time_series!(sys, Hour(1), Hour(1))
#transform_single_time_series!(sys2, Hour(1), Hour(1))
#enforced_region = ["1", "2", "3"] # first one is MRTO, second one is M2M coordinator, third one in RT is loopflow under M2M
enforced_region=["1","3","2"]
#enforced_region=["2","3","1"]
ptdf=PTDF(sys)

buildsubsystem(sys2, enforced_region, union(selected_line,monitoredlined_line))

exchange_1_2 = AreaInterchange(;
    name="1_2", available=true, active_power_flow=0.0, from_area=get_component(Area, sys, "1"), to_area=get_component(Area, sys, "2"),
    flow_limits=(from_to=99999, to_from=99999),
)
add_component!(sys, exchange_1_2)

exchange_1_3 = AreaInterchange(;
    name="1_3", available=true, active_power_flow=0.0, from_area=get_component(Area, sys, "1"), to_area=get_component(Area, sys, "3"),
    flow_limits=(from_to=99999, to_from=99999),
)
add_component!(sys, exchange_1_3)

exchange_2_3 = AreaInterchange(;
    name="2_3", available=true, active_power_flow=0.0, from_area=get_component(Area, sys, "2"), to_area=get_component(Area, sys, "3"),
    flow_limits=(from_to=99999, to_from=99999),
)
add_component!(sys, exchange_2_3)

exchange_1_2 = AreaInterchange(;
    name="1_2", available=true, active_power_flow=0.0, from_area=get_component(Area, sys2, "1"), to_area=get_component(Area, sys2, "2"),
    flow_limits=(from_to=99999, to_from=99999),
)
add_component!(sys2, exchange_1_2)

add_component_to_subsystem!(sys2, "a", exchange_1_2)
add_component_to_subsystem!(sys2, "b", exchange_1_2)

exchange_1_3 = AreaInterchange(;
    name="1_3", available=true, active_power_flow=0.0, from_area=get_component(Area, sys2, "1"), to_area=get_component(Area, sys2, "3"),
    flow_limits=(from_to=99999, to_from=99999),
)
add_component!(sys2, exchange_1_3)
add_component_to_subsystem!(sys2, "a", exchange_1_3)
add_component_to_subsystem!(sys2, "b", exchange_1_3)

exchange_2_3 = AreaInterchange(;
    name="2_3", available=true, active_power_flow=0.0, from_area=get_component(Area, sys2, "2"), to_area=get_component(Area, sys2, "3"),
    flow_limits=(from_to=99999, to_from=99999),
)
add_component!(sys2, exchange_2_3)
add_component_to_subsystem!(sys2, "a", exchange_2_3)
add_component_to_subsystem!(sys2, "b", exchange_2_3)

template_uc = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks=true))
set_device_model!(template_uc, ThermalStandard, ThermalBasicUnitCommitment)
set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
set_device_model!(template_uc, AreaInterchange, StaticBranch)

if use_monitoredline==0
    set_device_model!(template_uc,
    DeviceModel(Line, StaticBranchUnbounded; attributes=Dict("filter_function" => x -> get_name(x) in union(selected_line,monitoredlined_line)),
    ))
else
    set_device_model!(template_uc,
    DeviceModel(Line, StaticBranchUnbounded; attributes=Dict("filter_function" => x -> get_name(x) in union(selected_line,monitoredlined_line)),))

    set_device_model!(template_uc, DeviceModel(MonitoredLine, StaticBranchUnbounded, use_slacks = true))

    for b in monitoredlined_line
        line = PSY.get_component(Line, sys, b)
        PSY.convert_component!(sys, line, MonitoredLine)
    end
end

template_uc2 = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true), ["a", "b"])
set_device_model!(template_uc2, AreaInterchange, StaticBranch)
set_device_model!(template_uc2, ThermalStandard, ThermalBasicUnitCommitment)
set_device_model!(template_uc2, PowerLoad, StaticPowerLoad)

if use_monitoredline==0
    set_device_model!(template_uc2,
    DeviceModel(Line, StaticBranch; use_slacks=true, attributes=Dict("filter_function" => x -> get_name(x) in union(selected_line,monitoredlined_line)),))
    l = PSY.get_component(ACBranch, sys2, "A28")
    set_rating!(l,limit)
else
    set_device_model!(template_uc2,
    DeviceModel(Line, StaticBranchUnbounded; use_slacks=true, attributes=Dict("filter_function" => x -> get_name(x) in union(selected_line,monitoredlined_line)),))

    #set_device_model!(template_uc2, MonitoredLine, StaticBranch)
    set_device_model!(template_uc2, DeviceModel(MonitoredLine, StaticBranch, use_slacks = true))
    for b in monitoredlined_line
        line = PSY.get_component(Line, sys2, b)
        PSY.convert_component!(sys2, line, MonitoredLine)
    end
    l = PSY.get_component(MonitoredLine, sys2, "A28")
    set_flow_limits!(l,(from_to=limit,to_from=limit))
end
models = SimulationModels(;
    decision_models=[
        DecisionModel(
            template_uc,
            sys;
            name="UC0",
            optimizer=optimizer_with_attributes(
                Xpress.Optimizer,
                "MIPRELSTOP" => 0.00, # Set the relative mip gap tolerance
                "MAXMEMORYSOFT" => 600000,   # Set the maximum amount of memory the solver can use (in MB)
            ),
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
            optimizer=optimizer_with_attributes(
                Xpress.Optimizer,
                "MIPRELSTOP" => 0.00,       # Set the relative mip gap tolerance
                #"MAXMEMORYSOFT" => 600000,   # Set the maximum amount of memory the solver can use (in MB)
            ),
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

FVFF_area_interchange = FixValueFeedforward(;component_type=AreaInterchange,source=FlowActivePowerVariable,affected_values=[FlowActivePowerVariable],)
push!(uc_simulation_ff, FVFF_area_interchange)

#FVFF_moniterd_line = FixValueFeedforward(;
#    component_type=MonitoredLine, source=FlowActivePowerVariable,affected_values=[FlowActivePowerVariable],
#)
#push!(uc_simulation_ff, FVFF_moniterd_line)

#FVFF_line = FixValueFeedforward(;
#    component_type=Line, source=FlowActivePowerVariable,affected_values=[FlowActivePowerVariable],
#)
#push!(uc_simulation_ff, FVFF_line)

sequence = SimulationSequence(;
    models=models,
    feedforwards=Dict(
        "UC_Subsystem" => uc_simulation_ff,
    ),
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

uc0=models.decision_models[1].internal.container
uc2b=models.decision_models[2].internal.container.subproblems["b"]
uc2a=models.decision_models[2].internal.container.subproblems["a"]

println("obj uc0,uc2a,uc2b,",objective_value(uc0.JuMPmodel),",",objective_value(uc2a.JuMPmodel),",",objective_value(uc2b.JuMPmodel),",diff,",objective_value(uc0.JuMPmodel)-objective_value(uc2a.JuMPmodel)-objective_value(uc2b.JuMPmodel))

results = SimulationResults(sim)
results_uc = get_decision_problem_results(results, "UC0")
results_rt = get_decision_problem_results(results, "UC_Subsystem")

bus_df_uc0=read_bus_df(results_uc,"A28",0)
bus_df_uc2=read_bus_df(results_rt,"A28",1)

bus_df_uc0[!,"buscheck"]=bus_df_uc0[!,"ActivePowerBalance__ACBus"]-bus_df_uc0[!,"ThermalStandard"]/100-bus_df_uc0[!,"PowerLoad"]/100
println("UC0 bus check ActivePowerBalance__ACBus<>ThermalStandard/100-PowerLoad/100 ,",filter([:t,:buscheck] => (t,buscheck)-> (t>=1) && (abs(buscheck)>0.000001),  bus_df_uc0))
bus_df_uc2[!,"buscheck"]=bus_df_uc2[!,"ActivePowerBalance__ACBus"]-bus_df_uc2[!,"ThermalStandard"]/100-bus_df_uc2[!,"PowerLoad"]/100
println("UC2 bus check ActivePowerBalance__ACBus<>ThermalStandard/100-PowerLoad/100 ,",filter([:t,:buscheck] => (t,buscheck)-> (t>=1) && (abs(buscheck)>0.000001),  bus_df_uc2))
println("UC0 ActivePowerBalance__ACBus - UC2  StateEstimationInjections__ACBus",sum(bus_df_uc0[!,"ActivePowerBalance__ACBus"]-bus_df_uc2[!,"StateEstimationInjections__ACBus"]))
#println(filter([:t,:bus] => (t,bus)-> (t>=1) && (bus==203),  bus_df_uc2))

####### area flow check #########
areaflow_df0=combine(groupby(bus_df_uc0, [:t, :area]), [:flowcontribution, :loopflowcontribution] .=> sum)
areaflow1_df0=filter([:t,:area] => (t,area)-> (t>=1) && (area=="1"),  areaflow_df0)
areaflow2_df0=filter([:t,:area] => (t,area)-> (t>=1) && (area=="2"),  areaflow_df0)
areaflow3_df0=filter([:t,:area] => (t,area)-> (t>=1) && (area=="3"),  areaflow_df0)

areaflow_df2=combine(groupby(bus_df_uc2, [:t, :area]), [:flowcontribution, :loopflowcontribution] .=> sum)
areaflow1_df2=filter([:t,:area] => (t,area)-> (t>=1) && (area=="1"),  areaflow_df2)
areaflow2_df2=filter([:t,:area] => (t,area)-> (t>=1) && (area=="2"),  areaflow_df2)
areaflow3_df2=filter([:t,:area] => (t,area)-> (t>=1) && (area=="3"),  areaflow_df2)

sum(areaflow2_df0[!,"flowcontribution_sum"]-areaflow2_df2[!,"loopflowcontribution_sum"])
sum(areaflow3_df0[!,"flowcontribution_sum"]-areaflow3_df2[!,"loopflowcontribution_sum"])

println("areaflow1 check sum(uc0 flowcontribution_sum - uc2 loopflowcontribution_sum,",sum(areaflow1_df0[!,"flowcontribution_sum"]-areaflow1_df2[!,"loopflowcontribution_sum"]))
println("areaflow2 check sum(uc0 flowcontribution_sum - uc2 loopflowcontribution_sum,",sum(areaflow2_df0[!,"flowcontribution_sum"]-areaflow2_df2[!,"loopflowcontribution_sum"]))
println("areaflow3 check sum(uc0 flowcontribution_sum - uc2 loopflowcontribution_sum,",sum(areaflow3_df0[!,"flowcontribution_sum"]-areaflow3_df2[!,"loopflowcontribution_sum"]))

println("varflow_uc0,",value.(uc0.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:]) )
println("varflow_uc2a,",value.(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:]) )
println("var_flow_uc2b,",value.(uc2b.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:]) )

println(areaflow132_df[:,[:flow_subsystem_a ,:flow_subsystem_b, :seflow]])

areaflow13_df=innerjoin(areaflow1_df2, areaflow3_df2, on=:t,renamecols = "_1" => "_3")
areaflow132_df=innerjoin(areaflow13_df, areaflow2_df2, on=:t,renamecols = "" => "_2")

areaflow132_df[!,"flow_subsystem_a"]=areaflow132_df[!,"flowcontribution_sum_1"]+areaflow132_df[!,"flowcontribution_sum_3"]+areaflow132_df[!,"loopflowcontribution_sum_2"]
areaflow132_df[!,"flow_subsystem_b"]=areaflow132_df[!,"loopflowcontribution_sum_1"]+areaflow132_df[!,"loopflowcontribution_sum_3"]+areaflow132_df[!,"flowcontribution_sum_2"]
areaflow132_df[!,"seflow"]=areaflow132_df[!,"flowcontribution_sum_1"]+areaflow132_df[!,"flowcontribution_sum_3"]+areaflow132_df[!,"flowcontribution_sum_2"]

#StatsPlots.@df(areaflow132_df,Plots.plot(:t,[:flow_subsystem_a :flow_subsystem_b :seflow]))

areaflow13_df0=innerjoin(areaflow1_df0, areaflow3_df0, on=:t,renamecols = "_1" => "_3")
areaflow132_df0=innerjoin(areaflow13_df0, areaflow2_df0, on=:t,renamecols = "" => "_2")
areaflow132_df0[!,"flow_uc0"]=areaflow132_df0[!,"flowcontribution_sum_1"]+areaflow132_df0[!,"flowcontribution_sum_3"]+areaflow132_df0[!,"flowcontribution_sum_2"]
