using Pkg
#Pkg.activate("/projects/irtoc/ychen8/RTS/Multi-Stage/")
Pkg.activate(@__DIR__)
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
using HiGHS
# using Gurobi
using JuMP
using Logging
using Dates
using DataFrames

using HiGHS
#include("coordination_EnergyOnly.jl")
#include("GlobalM2M.jl")
#import StatsPlots
#import Plots

#using Plots
#gr()
const PSY = PowerSystems
const IF = InfrastructureSystems
const PSB = PowerSystemCaseBuilder

NT=5
# You may select some tielines if you don't want to add all tie lines. You need at least one tieline for each area pair
# You may set Line to be unbounded so that you don't have too many transmission constraint 
selected_line=["CA-1", "CB-1", "AB1"]  

# Additional lines to model. You may set it as bounded or unbounded.
monitoredlined_line=["A28"]
limit=0.2

sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
sys2 = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
transform_single_time_series!(sys, Hour(NT), Hour(NT))
transform_single_time_series!(sys2, Hour(NT), Hour(NT))
#transform_single_time_series!(sys, Hour(1), Hour(1))
#transform_single_time_series!(sys2, Hour(1), Hour(1))

function read_bus_df(results_rt,se=0)
    ac = read_realized_variable(results_rt, "ActivePowerVariable__ThermalStandard")
    ab = read_realized_variable(results_rt, "ActivePowerBalance__ACBus")
    if se==1  al = read_realized_variable(results_rt, "StateEstimationInjections__ACBus") end
    ald = read_realized_variable(results_rt, "ActivePowerTimeSeriesParameter__PowerLoad")
    am = read_realized_variable(results_rt, "FlowActivePowerVariable__MonitoredLine")

    ac0 = read_realized_variable(results_uc, "ActivePowerVariable__ThermalStandard")
    ab0 = read_realized_variable(results_uc, "ActivePowerBalance__ACBus")
    ald0 = read_realized_variable(results_uc, "ActivePowerTimeSeriesParameter__PowerLoad")
    am0 = read_realized_variable(results_uc, "FlowActivePowerVariable__MonitoredLine")

    bus_ActivePowerVariable__ThermalStandard=Dict()
    bus_ActivePowerTimeSeriesParameter__PowerLoad=Dict()
    bus_ActivePowerBalance__ACBus=Dict()
    bus_StateEstimationInjections__ACBus=Dict()

    #101,[-37.327065809684214, -37.353785256, -37.9549728]
    bus_df=DataFrames.DataFrame(;
    t =Int64[], bus=[], area = [], ThermalStandard=[], PowerLoad=[], ActivePowerBalance__ACBus=[],
    StateEstimationInjections__ACBus=[], ptdf=[], flowcontribution=[], loopflowcontribution=[]
    )
    for t in 1:NT
        for b in PSY.get_components(PSY.Bus,sys)
            bn=get_number(b); bus_ActivePowerVariable__ThermalStandard[t,bn]=0
            bus_ActivePowerTimeSeriesParameter__PowerLoad[t,bn]=0; bus_ActivePowerBalance__ACBus[t,bn]=0
            bus_StateEstimationInjections__ACBus[t,bn]=0
        end
    end

    for i in names(ab)
        try b=parse(Int,i)
            for t=1:NT bus_ActivePowerBalance__ACBus[t,b]=ab[t,i] end
        catch e println("error= ",e,",i=",i)  end        
    end
    if se==1
        for i in names(al)
            try b=parse(Int,i)
                for t=1:NT bus_StateEstimationInjections__ACBus[t,b]=al[t,i] end
            catch e println("error= ",e,",i=",i) end     
        end
    end    

    for i in names(ald)
        try b=get_number(get_bus(get_component(PowerLoad, sys, i)))
            for t=1:NT  bus_ActivePowerTimeSeriesParameter__PowerLoad[t,b]=bus_ActivePowerTimeSeriesParameter__PowerLoad[t,b]+ald[t,i] end  
        catch e println("error= ",e,",i=",i) end 
    end

    for i in names(ac)
        try b=get_number(get_bus(get_component(ThermalStandard, sys, i)))
        for t=1:NT bus_ActivePowerVariable__ThermalStandard[t,b]=bus_ActivePowerVariable__ThermalStandard[t,b]+ac[t,i] end  
        catch e println("error ",e,",i=",i) end 
    end

    for t in 1:NT
        for b in PSY.get_components(PSY.Bus,sys)
            bn=get_number(b); area=get_name(get_area(b)); 
            push!(bus_df,(t,bn,area, bus_ActivePowerVariable__ThermalStandard[t,bn], bus_ActivePowerTimeSeriesParameter__PowerLoad[t,bn],
            bus_ActivePowerBalance__ACBus[t,bn], bus_StateEstimationInjections__ACBus[t,bn], ptdf[line,bn],
            bus_ActivePowerBalance__ACBus[t,bn]*ptdf[line,bn],bus_StateEstimationInjections__ACBus[t,bn]*ptdf[line,bn]))
        end
    end

    bus_df[!,"buscheck"]=bus_df[!,"ActivePowerBalance__ACBus"]-bus_df[!,"ThermalStandard"]/100-bus_df[!,"PowerLoad"]/100
    println("bus check ActivePowerBalance__ACBus<>ThermalStandard/100-PowerLoad/100 ,",filter([:t,:buscheck] => (t,buscheck)-> (t>=1) && (abs(buscheck)>0.000001),  bus_df))
    #println(filter([:t,:bus] => (t,bus)-> (t>=1) && (bus==203),  bus_df))
    return(bus_df)
end    

function check_models(uc0,uc2a)
    sol0=Dict()
    v0=uc0.variables[PowerSimulations.VariableKey{ActivePowerVariable, ThermalStandard}("")]
    sol0=Dict(zip(name.(v0),value.(v0)))
    for v in v0
        fix(v,value.(v),force=true)
    end    

    v01=uc0.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, AreaInterchange}("")]
    for v in v01
        fix(v,value.(v),force=true)
    end  
    
    v2a=uc2a.variables[PowerSimulations.VariableKey{ActivePowerVariable, ThermalStandard}("")]
    sol2a=Dict(zip(name.(v2a),value.(v2a)))

    for (k,v) in sol2a
        if abs(sol2a[k]-sol0[k])>0.00001
            println("k,",k,",",sol2a[k],",",sol0[k])
            var0=variable_by_name(uc0.JuMPmodel,k)
            fix(var0,sol2a[k],force=true)
        end    
    end  
    solve!(models.decision_models[1])
end

function objterms(uc0,ff)
    objd0=Dict(); sol0=Dict(); objv0=0
    obj0=objective_function(uc0.JuMPmodel); objd0=Dict(zip(name.(keys(obj0.terms)),values(obj0.terms))); 
    av0 = JuMP.all_variables(uc0.JuMPmodel); sol0=Dict(zip(name.(av0), value.(av0)))

    open(ff,"w") do io
        redirect_stdout(io) do
            for (k,v) in objd0
                objv0=objv0+sol0[k]*objd0[k]
                println("name;value;objcoef;obj_contribution;",k,";",sol0[k],";",objd0[k],";", sol0[k]*objd0[k])      
            end
        end
    end  
    return(objd0,sol0, objv0)  
end    

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

exchange_1_3 = AreaInterchange(;
    name="1_3", available=true, active_power_flow=0.0, from_area=get_component(Area, sys2, "1"), to_area=get_component(Area, sys2, "3"),
    flow_limits=(from_to=99999, to_from=99999),
)
add_component!(sys2, exchange_1_3)

exchange_2_3 = AreaInterchange(;
    name="2_3", available=true, active_power_flow=0.0, from_area=get_component(Area, sys2, "2"), to_area=get_component(Area, sys2, "3"),
    flow_limits=(from_to=99999, to_from=99999),
)
add_component!(sys2, exchange_2_3)

# AREAPTDF: 
#   each area has its own power balance equation; 
#   transmission constraints LHS include variables from all areas
template_uc = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks=true))
set_device_model!(template_uc, ThermalStandard, ThermalBasicUnitCommitment)
set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
set_device_model!(template_uc, AreaInterchange, StaticBranch)

set_device_model!(template_uc,
DeviceModel(Line, StaticBranchUnbounded; attributes=Dict("filter_function" => x -> get_name(x) in union(selected_line,monitoredlined_line)),))
set_device_model!(template_uc, DeviceModel(MonitoredLine, StaticBranchUnbounded, use_slacks = true))
for b in monitoredlined_line
    line = PSY.get_component(Line, sys, b)
    PSY.convert_component!(sys, line, MonitoredLine)
end

#template_uc2 = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true), ["a", "b"])
template_uc2 = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks=true))
set_device_model!(template_uc2, AreaInterchange, StaticBranch)
set_device_model!(template_uc2, ThermalStandard, ThermalBasicUnitCommitment)
set_device_model!(template_uc2, PowerLoad, StaticPowerLoad)

set_device_model!(template_uc2,
    DeviceModel(Line, StaticBranchUnbounded; use_slacks=true, attributes=Dict("filter_function" => x -> get_name(x) in union(selected_line,monitoredlined_line)),))

set_device_model!(template_uc2, DeviceModel(MonitoredLine, StaticBranch, use_slacks = true))
for b in monitoredlined_line
    line = PSY.get_component(Line, sys2, b)
    PSY.convert_component!(sys2, line, MonitoredLine)
end
l = PSY.get_component(MonitoredLine, sys2, "A28")
set_flow_limits!(l,(from_to=limit,to_from=limit))

models = SimulationModels(;
    decision_models=[
        DecisionModel(
            template_uc,
            sys;
            name="UC0",
            optimizer=optimizer_with_attributes(
                HiGHS.Optimizer,
                # "MIPGap" => 0.01,
                # "TimeLimit" => 3000,
                #"MIPRELSTOP" => 0.00, # Set the relative mip gap tolerance
                #"MAXMEMORYSOFT" => 600000,   # Set the maximum amount of memory the solver can use (in MB)
            ),
            system_to_file=false,
            optimizer_solve_log_print=false,
            direct_mode_optimizer=true,
            store_variable_names=true,
            calculate_conflict=true,
        ),
        DecisionModel(
#            MultiRegionProblem,
            template_uc2,
            sys2;
            name="UC_Subsystem",
            optimizer=optimizer_with_attributes(
                HiGHS.Optimizer,
                # "MIPRELSTOP" => 0.00,       # Set the relative mip gap tolerance
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

#feed forward area interchange
FVFF_area_interchange = FixValueFeedforward(;component_type=AreaInterchange,source=FlowActivePowerVariable,affected_values=[FlowActivePowerVariable],)
push!(uc_simulation_ff, FVFF_area_interchange)

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
uc2=models.decision_models[2].internal.container

println("obj uc0,uc2,",objective_value(uc0.JuMPmodel),",",objective_value(uc2.JuMPmodel),",",objective_value(uc2b.JuMPmodel),",diff,",objective_value(uc0.JuMPmodel)-objective_value(uc2.JuMPmodel))

######### Model check ############
open("mod_uc0.txt","w") do io
    redirect_stdout(io) do
        println(objective_function(uc0.JuMPmodel))
        for k in all_constraints(uc0.JuMPmodel,; include_variable_in_set_constraints = true)           
            println(name(k),",",k) 
        end    
    end
end
open("mod_uc2.txt","w") do io
    redirect_stdout(io) do
        println(objective_function(uc2.JuMPmodel))
        for k in all_constraints(uc2.JuMPmodel,; include_variable_in_set_constraints = true)           
            println(name(k),",",k) 
        end    
    end
end

for (k,v) in uc0.constraints println(k) end
uc0.constraints[InfrastructureSystems.Optimization.ConstraintKey{CopperPlateBalanceConstraint, Area}("")][:,1]  

for (k,v) in uc2.constraints println(k) end
uc2.constraints[InfrastructureSystems.Optimization.ConstraintKey{CopperPlateBalanceConstraint, Area}("")][:,1]  

value.(uc0.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, AreaInterchange}("")][:,1])
fix_value.(uc2.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, AreaInterchange}("")][:,1])
##########################

results = SimulationResults(sim)
results_uc0 = get_decision_problem_results(results, "UC0")
results_uc1 = get_decision_problem_results(results, "UC_Subsystem")

bus_df_uc0=read_bus_df(results_uc0,0)
bus_df_uc2=read_bus_df(results_uc1,0)

bus_df_uc0[!,"buscheck"]=bus_df_uc0[!,"ActivePowerBalance__ACBus"]-bus_df_uc0[!,"ThermalStandard"]/100-bus_df_uc0[!,"PowerLoad"]/100
println("UC0 bus check ActivePowerBalance__ACBus<>ThermalStandard/100-PowerLoad/100 ,",filter([:t,:buscheck] => (t,buscheck)-> (t>=1) && (abs(buscheck)>0.000001),  bus_df_uc0))
bus_df_uc2[!,"buscheck"]=bus_df_uc2[!,"ActivePowerBalance__ACBus"]-bus_df_uc2[!,"ThermalStandard"]/100-bus_df_uc2[!,"PowerLoad"]/100
println("UC2 bus check ActivePowerBalance__ACBus<>ThermalStandard/100-PowerLoad/100 ,",filter([:t,:buscheck] => (t,buscheck)-> (t>=1) && (abs(buscheck)>0.000001),  bus_df_uc2))
#println(filter([:t,:bus] => (t,bus)-> (t>=1) && (bus==203),  bus_df_uc2))

###### area MW check #########
area_df0=combine(groupby(bus_df_uc0, [:t, :area]), [:ThermalStandard, :PowerLoad, :ActivePowerBalance__ACBus, :StateEstimationInjections__ACBus] .=> sum)
area_df0[!,"areacheck"]=area_df0[!,"ActivePowerBalance__ACBus_sum"]-area_df0[!,"ThermalStandard_sum"]/100-area_df0[!,"PowerLoad_sum"]/100
sum(area_df0[!,"areacheck"])

area1_df0=filter([:t,:area] => (t,area)-> (t>=1) && (area=="1"),  area_df0)
area2_df0=filter([:t,:area] => (t,area)-> (t>=1) && (area=="2"),  area_df0)
area3_df0=filter([:t,:area] => (t,area)-> (t>=1) && (area=="3"),  area_df0)

area_df2=combine(groupby(bus_df_uc2, [:t, :area]), [:ThermalStandard, :PowerLoad, :ActivePowerBalance__ACBus, :StateEstimationInjections__ACBus] .=> sum)
area_df2[!,"areacheck"]=area_df2[!,"ActivePowerBalance__ACBus_sum"]-area_df2[!,"ThermalStandard_sum"]/100-area_df2[!,"PowerLoad_sum"]/100
sum(area_df2[!,"areacheck"])

area1_df2=filter([:t,:area] => (t,area)-> (t>=1) && (area=="1"),  area_df2)
area2_df2=filter([:t,:area] => (t,area)-> (t>=1) && (area=="2"),  area_df2)
area3_df2=filter([:t,:area] => (t,area)-> (t>=1) && (area=="3"),  area_df2)

println("area1 check uc0-uc2,", sum(area1_df2[!,"ActivePowerBalance__ACBus_sum"]-area1_df0[!,"ActivePowerBalance__ACBus_sum"]))
println("area2 check uc0-uc2,", sum(area2_df2[!,"ActivePowerBalance__ACBus_sum"]-area2_df0[!,"ActivePowerBalance__ACBus_sum"]))
println("area3 check uc0-uc2,", sum(area3_df2[!,"ActivePowerBalance__ACBus_sum"]-area3_df0[!,"ActivePowerBalance__ACBus_sum"]))

####### area flow check #########
areaflow_df0=combine(groupby(bus_df_uc0, [:t, :area]), [:flowcontribution, :loopflowcontribution] .=> sum)
areaflow1_df0=filter([:t,:area] => (t,area)-> (t>=1) && (area=="1"),  areaflow_df0)
areaflow2_df0=filter([:t,:area] => (t,area)-> (t>=1) && (area=="2"),  areaflow_df0)
areaflow3_df0=filter([:t,:area] => (t,area)-> (t>=1) && (area=="3"),  areaflow_df0)

areaflow_df2=combine(groupby(bus_df_uc2, [:t, :area]), [:flowcontribution, :loopflowcontribution] .=> sum)
areaflow1_df2=filter([:t,:area] => (t,area)-> (t>=1) && (area=="1"),  areaflow_df2)
areaflow2_df2=filter([:t,:area] => (t,area)-> (t>=1) && (area=="2"),  areaflow_df2)
areaflow3_df2=filter([:t,:area] => (t,area)-> (t>=1) && (area=="3"),  areaflow_df2)

println("areaflow1 check sum(uc0 flowcontribution_sum - uc2 flowcontribution_sum,",sum(areaflow1_df0[!,"flowcontribution_sum"]-areaflow1_df2[!,"flowcontribution_sum"]))
println("areaflow2 check sum(uc0 flowcontribution_sum - uc2 flowcontribution_sum,",sum(areaflow2_df0[!,"flowcontribution_sum"]-areaflow2_df2[!,"flowcontribution_sum"]))
println("areaflow3 check sum(uc0 flowcontribution_sum - uc2 flowcontribution_sum,",sum(areaflow3_df0[!,"flowcontribution_sum"]-areaflow3_df2[!,"flowcontribution_sum"]))

areaflow1_df0[!,"seflow"]=areaflow1_df0[!,"flowcontribution_sum"]+areaflow2_df0[!,"flowcontribution_sum"]+areaflow3_df0[!,"flowcontribution_sum"]
areaflow1_df2[!,"seflow"]=areaflow1_df2[!,"flowcontribution_sum"]+areaflow2_df2[!,"flowcontribution_sum"]+areaflow3_df2[!,"flowcontribution_sum"]

#matches solved flow
println("varflow_uc0,",value.(uc0.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:]) )
println("varflow_uc2,",value.(uc2.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:]) )



