using Pkg
Pkg.activate(@__DIR__)
#Pkg.add("Xpress")
using Revise

#ENV["XPRESSDIR"] = "C:\\xpressmp"
#ENV["XPAUTH_PATH"] = "C:\\xpressmp\\bin"

using HydroPowerSimulations
using PowerSimulations
using PowerSystemCaseBuilder
using InfrastructureSystems
using StorageSystemsSimulations
using PowerSimulationsDecomposition
using PowerSystems
# using PowerGraphics

#using Xpress
using JuMP
using Logging
using Dates
using DataFrames
import ShiftedArrays: lead, lag
using Gurobi
using HiGHS

include("YC_test_function.jl")
#include("coordination_EnergyOnly.jl")
#include("GlobalM2M.jl")
#import StatsPlots
#import Plots

#using Plots
#gr()
const PSY = PowerSystems
const IF = InfrastructureSystems
const PSB = PowerSystemCaseBuilder
const PSI = PowerSimulations

NT=24
    selected_line=["CA-1", "CB-1", "AB1"]
    monitoredlined_line=["A28"]
    limit=0.2
    use_monitoredline=1

    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    transform_single_time_series!(sys, Hour(NT), Hour(NT))
    transform_single_time_series!(sys2, Hour(1), Hour(1))
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
        #DeviceModel(Line, StaticBranchUnbounded; attributes=Dict("filter_function" => x -> get_name(x) in selected_line),
        #DeviceModel(Line, StaticBranch; attributes=Dict("filter_function" => x -> get_name(x) in selected_line),
        ))
    else
        set_device_model!(template_uc,
        DeviceModel(Line, StaticBranchUnbounded; attributes=Dict("filter_function" => x -> get_name(x) in union(selected_line,monitoredlined_line)),))

        set_device_model!(template_uc, MonitoredLine, StaticBranch)
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

        set_device_model!(template_uc2, MonitoredLine, StaticBranch)
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
                    HiGHS.Optimizer,
                #    Xpress.Optimizer,
                #    "MIPRELSTOP" => 0.00, # Set the relative mip gap tolerance
                #    "MAXMEMORYSOFT" => 600000,   # Set the maximum amount of memory the solver can use (in MB)
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
                    HiGHS.Optimizer,
                    #Xpress.Optimizer,
                    #"MIPRELSTOP" => 0.00,       # Set the relative mip gap tolerance
                    #"MAXMEMORYSOFT" => 600000,   # Set the maximum amount of memory the solver can use (in MB)
                ),
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

    #FVFF_area_interchange = FixValueFeedforward(;component_type=AreaInterchange,source=FlowActivePowerVariable,affected_values=[FlowActivePowerVariable],)
    #push!(uc_simulation_ff, FVFF_area_interchange)

    #LBFF = FixValueFeedforward(;
    #    component_type=ThermalStandard,
    #    source=OnVariable,
    #    affected_values=[OnVariable],
    #)
    #push!(uc_simulation_ff, LBFF)

    sequence = SimulationSequence(;
        models=models,
        feedforwards=Dict(
            "UC_Subsystem" => uc_simulation_ff,
        ),
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

    uc0=models.decision_models[1].internal.container
    uc2b=models.decision_models[2].internal.container.subproblems["b"]
    uc2a=models.decision_models[2].internal.container.subproblems["a"]
    println("obj uc0,uc2a,uc2b,",objective_value(uc0.JuMPmodel),",",objective_value(uc2a.JuMPmodel),",",objective_value(uc2b.JuMPmodel),",diff,",objective_value(uc0.JuMPmodel)-objective_value(uc2a.JuMPmodel)-objective_value(uc2b.JuMPmodel))

    results = SimulationResults(sim)
    # #results = SimulationResults(sim)
    results_uc = get_decision_problem_results(results, "UC0")
    results_ha = get_decision_problem_results(results, "UC_Subsystem")
    aa = read_realized_variable(results_uc, "FlowActivePowerVariable__MonitoredLine")
    ac = read_realized_variable(results_ha, "FlowActivePowerVariable__MonitoredLine")


area_df=DataFrames.DataFrame(;
t =Int64[], uc=[], area1_gen = [], area1_load = [], area2_gen = [], area2_load = [], 
            area3_gen = [], area3_load = [],   
)

results_uc = get_decision_problem_results(results, "UC0")
results_rt = get_decision_problem_results(results, "UC_Subsystem")

bus_df=read_bus_df(results_rt,"A28",1)
#println(filter([:t,:bus] => (t,bus)-> (t>=1) && (bus==203),  bus_df))

####### area MW check #########
area_df=combine(groupby(bus_df, [:t, :area]), [:ThermalStandard, :PowerLoad, :ActivePowerBalance__ACBus, :StateEstimationInjections__ACBus] .=> sum)
area_df[!,"areacheck"]=area_df[!,"ActivePowerBalance__ACBus_sum"]-area_df[!,"ThermalStandard_sum"]/100-area_df[!,"PowerLoad_sum"]/100

area1_df=filter([:t,:area] => (t,area)-> (t>=1) && (area=="1"),  area_df)
area2_df=filter([:t,:area] => (t,area)-> (t>=1) && (area=="2"),  area_df)
area3_df=filter([:t,:area] => (t,area)-> (t>=1) && (area=="3"),  area_df)

area1_df[!,"secheck"]=lag(area1_df[!,"ActivePowerBalance__ACBus_sum"])-area1_df[!,"StateEstimationInjections__ACBus_sum"]
area2_df[!,"secheck"]=lag(area2_df[!,"ActivePowerBalance__ACBus_sum"])-area2_df[!,"StateEstimationInjections__ACBus_sum"]
area3_df[!,"secheck"]=lag(area3_df[!,"ActivePowerBalance__ACBus_sum"])-area3_df[!,"StateEstimationInjections__ACBus_sum"]

println("area1 check max diff lag(ActivePowerBalance__ACBus_sum)-StateEstimationInjections__ACBus_sum,", maximum(area1_df[!,"secheck"][2,:]))
println("area2 check max diff lag(ActivePowerBalance__ACBus_sum)-StateEstimationInjections__ACBus_sum,", maximum(area2_df[!,"secheck"][2,:]))
println("area3 check max diff lag(ActivePowerBalance__ACBus_sum)-StateEstimationInjections__ACBus_sum,", maximum(area3_df[!,"secheck"][2,:]))
##########################

####### area flow check #########
areaflow_df=combine(groupby(bus_df, [:t, :area]), [:flowcontribution, :loopflowcontribution] .=> sum)
#areaflow_df[!,"areacheck"]=area_df[!,"ActivePowerBalance__ACBus_sum"]-area_df[!,"ThermalStandard_sum"]/100-area_df[!,"PowerLoad_sum"]/100

areaflow1_df=filter([:t,:area] => (t,area)-> (t>=1) && (area=="1"),  areaflow_df)
areaflow2_df=filter([:t,:area] => (t,area)-> (t>=1) && (area=="2"),  areaflow_df)
areaflow3_df=filter([:t,:area] => (t,area)-> (t>=1) && (area=="3"),  areaflow_df)

areaflow1_df[!,"secheck"]=lag(areaflow1_df[!,"flowcontribution_sum"])-areaflow1_df[!,"loopflowcontribution_sum"]
areaflow2_df[!,"secheck"]=lag(areaflow2_df[!,"flowcontribution_sum"])-areaflow2_df[!,"loopflowcontribution_sum"]
areaflow3_df[!,"secheck"]=lag(areaflow3_df[!,"flowcontribution_sum"])-areaflow3_df[!,"loopflowcontribution_sum"]

println("areaflow1 check max diff lag(flowcontribution_sum)-loopflowcontribution_sum,", maximum(area1_df[!,"secheck"][2,:]))
println("areaflow2 check max diff lag(flowcontribution_sum)-loopflowcontribution_sum,", maximum(area2_df[!,"secheck"][2,:]))
println("areaflow3 check max diff lag(flowcontribution_sum)-loopflowcontribution_sum,", maximum(area3_df[!,"secheck"][2,:]))
##########################

areaflow13_df=innerjoin(areaflow1_df, areaflow3_df, on=:t,renamecols = "_1" => "_3")
areaflow132_df=innerjoin(areaflow13_df, areaflow2_df, on=:t,renamecols = "" => "_2")

areaflow132_df[!,"flow_subsystem_a"]=areaflow132_df[!,"flowcontribution_sum_1"]+areaflow132_df[!,"flowcontribution_sum_3"]+areaflow132_df[!,"loopflowcontribution_sum_2"]
areaflow132_df[!,"flow_subsystem_b"]=areaflow132_df[!,"loopflowcontribution_sum_1"]+areaflow132_df[!,"loopflowcontribution_sum_3"]+areaflow132_df[!,"flowcontribution_sum_2"]
areaflow132_df[!,"seflow"]=areaflow132_df[!,"flowcontribution_sum_1"]+areaflow132_df[!,"flowcontribution_sum_3"]+areaflow132_df[!,"flowcontribution_sum_2"]

StatsPlots.@df(areaflow132_df,Plots.plot(:t,[:flow_subsystem_a :flow_subsystem_b :seflow]))

