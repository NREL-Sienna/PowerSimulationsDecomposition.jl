using Pkg
#Pkg.activate(".")
Pkg.activate(@__DIR__)
#run(`bash -c "module load gurobi"`)
#=
ENV["GUROBI_HOME"] = "/nopt/nrel/apps/software/gurobi/gurobi1100/linux64"
ENV["PATH"] *= ":$ENV[\"GUROBI_HOME\"]/bin"
ENV["GRB_LICENSE_FILE"] = "/nopt/nrel/apps/software/gurobi/tlicense/gurobi.lic"  
=#
ENV["XPRESSDIR"] = "C:\\xpressmp"
ENV["XPAUTH_PATH"] = "C:\\xpressmp\\bin"
using PowerSystems
using PowerSimulations
using PowerSystemCaseBuilder
using PowerNetworkMatrices
using HydroPowerSimulations
using StorageSystemsSimulations
using JuMP
using Dates
#using Gurobi
using PowerSimulationsDecomposition
using Revise
using Logging
using CSV
using DataFrames
using Xpress
using HiGHS
using InfrastructureSystems

const HPS = HydroPowerSimulations
const PSI = PowerSimulations
const SSS = StorageSystemsSimulations
const PSY = PowerSystems

function write_cons_coeff(uc2a,con2a,fnm)
    open(fnm,"w") do io
        redirect_stdout(io) do
            for k in all_variables(uc2a.JuMPmodel)  
                if normalized_coefficient(con2a, k)!=0         
                    println("coef;",name(k),";",normalized_coefficient(con2a, k), ";is_fixed;",is_fixed(k),";value;",value(k))
                end  
            end 
        end
    end
end

function write_model(uc0,fnm)
    open(fnm,"w") do io
        redirect_stdout(io) do
            println(objective_function(uc0.JuMPmodel))
            for k in all_constraints(uc0.JuMPmodel,; include_variable_in_set_constraints = true)           
                println(name(k),",",k) 
            end    
        end    
    end
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

function read_bus_df(results_rt,line="",se=0,standardload=0)
    ac = read_realized_variable(results_rt, "ActivePowerVariable__ThermalStandard")
    ac1 = read_realized_variable(results_rt, "ActivePowerVariable__RenewableDispatch")
    ac2 = read_realized_variable(results_rt, "ActivePowerVariable__HydroDispatch")

    ab = read_realized_variable(results_rt, "ActivePowerBalance__ACBus")
    if se==1  al = read_realized_variable(results_rt, "StateEstimationInjections__ACBus") end
    if standardload==1
        ald = read_realized_variable(results_rt, "ActivePowerTimeSeriesParameter__StandardLoad")
    else
        ald = read_realized_variable(results_rt, "ActivePowerTimeSeriesParameter__PowerLoad")
    end        
    if line!="" am = read_realized_variable(results_rt, "FlowActivePowerVariable__MonitoredLine") end
    ahvdc=[]
    try 
        ahvdc=read_realized_variable(results_rt, "FlowActivePowerVariable__TwoTerminalHVDCLine")
    catch e end

    bus_ActivePowerVariable__ThermalStandard=Dict()
    bus_ActivePowerVariable__RenewableDispatch=Dict()
    bus_ActivePowerVariable__HydroDispatch=Dict()
    bus_ActivePowerTimeSeriesParameter__PowerLoad=Dict()
    bus_ActivePowerBalance__ACBus=Dict()
    bus_StateEstimationInjections__ACBus=Dict()
    bus_ActivePowerVariable__HVDC=Dict()

    bus_df=DataFrames.DataFrame(;
    t =Int64[], bus=[], area = [], ThermalStandard=[],Renewable=[],Hydro=[],
    PowerLoad=[], HVDC=[],ActivePowerBalance__ACBus=[], StateEstimationInjections__ACBus=[], 
    ptdf=[], flowcontribution=[], loopflowcontribution=[]
    )
    NT=size(ac,1)
    for t in 1:NT
        for b in PSY.get_components(PSY.Bus,sys)
            bn=get_number(b); bus_ActivePowerVariable__ThermalStandard[t,bn]=0
            bus_ActivePowerVariable__RenewableDispatch[t,bn]=0
            bus_ActivePowerVariable__HydroDispatch[t,bn]=0
            bus_ActivePowerVariable__HVDC[t,bn]=0
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
        if standardload==1
            try b=get_number(get_bus(get_component(StandardLoad, sys, i)))
                for t=1:NT  bus_ActivePowerTimeSeriesParameter__PowerLoad[t,b]=bus_ActivePowerTimeSeriesParameter__PowerLoad[t,b]+ald[t,i] end  
            catch e println("error= ",e,",i=",i) end 
        else
            try b=get_number(get_bus(get_component(PowerLoad, sys, i)))
                for t=1:NT  bus_ActivePowerTimeSeriesParameter__PowerLoad[t,b]=bus_ActivePowerTimeSeriesParameter__PowerLoad[t,b]+ald[t,i] end  
            catch e println("error= ",e,",i=",i) end 
        end                    
    end

    if length(ahvdc)>0
        for i in names(ahvdc)
            try b=get_number(get_from(get_arc(get_component(TwoTerminalHVDCLine, sys, i))))
            for t=1:NT 
                #bus_ActivePowerVariable__ThermalStandard[t,b]=bus_ActivePowerVariable__ThermalStandard[t,b]-ahvdc[t,i] 
                bus_ActivePowerVariable__HVDC[t,b]=bus_ActivePowerVariable__HVDC[t,b]-ahvdc[t,i] 
            end  
            catch e println("error ",e,",i=",i) end 
            try b=get_number(get_to(get_arc(get_component(TwoTerminalHVDCLine, sys, i))))
                for t=1:NT 
                    #bus_ActivePowerVariable__ThermalStandard[t,b]=bus_ActivePowerVariable__ThermalStandard[t,b]+ahvdc[t,i] 
                    bus_ActivePowerVariable__HVDC[t,b]=bus_ActivePowerVariable__HVDC[t,b]+ahvdc[t,i] 
                end  
            catch e println("error ",e,",i=",i) end 
        end
    end    
    
    for i in names(ac)
        try b=get_number(get_bus(get_component(ThermalStandard, sys, i)))
        for t=1:NT bus_ActivePowerVariable__ThermalStandard[t,b]=bus_ActivePowerVariable__ThermalStandard[t,b]+ac[t,i] end  
        catch e println("error ",e,",i=",i) end 
    end
    for i in names(ac1)
        try b=get_number(get_bus(get_component(RenewableDispatch, sys, i)))
        for t=1:NT bus_ActivePowerVariable__RenewableDispatch[t,b]=bus_ActivePowerVariable__RenewableDispatch[t,b]+ac1[t,i] end  
        catch e println("error ",e,",i=",i) end 
    end
    for i in names(ac2)
        try b=get_number(get_bus(get_component(HydroDispatch, sys, i)))
        for t=1:NT bus_ActivePowerVariable__HydroDispatch[t,b]=bus_ActivePowerVariable__HydroDispatch[t,b]+ac2[t,i] end  
        catch e println("error ",e,",i=",i) end 
    end

    for t in 1:NT
        for b in PSY.get_components(PSY.Bus,sys)
            bn=get_number(b); area=get_name(get_area(b)); 
            if line=="" gsf=0 else gsf=ptdf[line,bn] end 
            push!(bus_df,(t,bn,area, bus_ActivePowerVariable__ThermalStandard[t,bn], 
            bus_ActivePowerVariable__RenewableDispatch[t,bn],bus_ActivePowerVariable__HydroDispatch[t,bn],
            bus_ActivePowerTimeSeriesParameter__PowerLoad[t,bn],bus_ActivePowerVariable__HVDC[t,bn],
            bus_ActivePowerBalance__ACBus[t,bn], bus_StateEstimationInjections__ACBus[t,bn], gsf,
            bus_ActivePowerBalance__ACBus[t,bn]*gsf,bus_StateEstimationInjections__ACBus[t,bn]*gsf))
        end
    end

    bus_df[!,"buscheck"]=bus_df[!,"ActivePowerBalance__ACBus"]-bus_df[!,"ThermalStandard"]/100-bus_df[!,"Renewable"]/100-bus_df[!,"Hydro"]/100-bus_df[!,"PowerLoad"]/100
    println("bus check ActivePowerBalance__ACBus<>ThermalStandard/100-PowerLoad/100 ,",filter([:t,:buscheck] => (t,buscheck)-> (t>=1) && (abs(buscheck)>0.000001),  bus_df))
    #println(filter([:t,:bus] => (t,bus)-> (t>=1) && (bus==203),  bus_df))
    return(bus_df)
end    

function buildsubsystem1(sys, area_region_dict, selected_line, monitoredline_subsys=[])
    subsys=unique(values(area_region_dict))

    for b in PSY.get_components(PSY.AreaInterchange, sys)
        println(b); 
        for v in subsys
            add_component_to_subsystem!(sys, v, b) 
        end
    end

    for b in PSY.get_components(PSY.Area, sys)
        region=area_region_dict[get_name(b)]
        PSY.set_ext!(b, Dict("subregion" => Set([region])))
        add_component_to_subsystem!(sys, region, b)
    end
    for b in PSY.get_components(PSY.StaticInjection, sys)
        region=area_region_dict[get_name(get_area(get_bus(b)))]
        PSY.set_ext!(b, Dict("subregion" => Set([region])))
        add_component_to_subsystem!(sys, region, b)
    end
    for b in PSY.get_components(PSY.Bus, sys)
        region=area_region_dict[get_name(get_area(b))]
        PSY.set_ext!(b, Dict("subregion" => Set([region])))
        add_component_to_subsystem!(sys, region, b)
    end
    
    if length(selected_line)>0
        for b in selected_line 
            l=PSY.get_component(ACBranch, sys,b)
            for v in subsys
                PSY.set_ext!(l, Dict("subregion" => Set([v])))
                add_component_to_subsystem!(sys, v, l) 
            end            
        end  
    end    
    
    if length(monitoredline_subsys) >0
        for (k,s) in monitoredline_subsys 
            l=PSY.get_component(ACBranch, sys,k)
            for v in s
                PSY.set_ext!(l, Dict("subregion" => Set([v])))
                add_component_to_subsystem!(sys, v, l) 
            end            
        end  
    end

    for dc in get_components(TwoTerminalHVDCLine, sys)
        tbus=get_to(get_arc(dc)); fbus=get_from(get_arc(dc))
        tregion=area_region_dict[get_name(get_area(tbus))]
        fregion=area_region_dict[get_name(get_area(fbus))]
        PSY.set_ext!(dc, Dict("subregion" => Set([tregion])))
        add_component_to_subsystem!(sys, tregion, dc) 
        
        #Option 1, add HVDC also to fregion. Can cause double counting of fixed and dispatchable HVDC MW
        #if fregion!=tregion
        #    PSY.set_ext!(dc, Dict("subregion" => Set([fregion])))
        #    add_component_to_subsystem!(sys, fregion, dc)
        #end                

        #Option 2, add fbus to tregion
        #if fregion!=tregion
        #    PSY.set_ext!(fbus, Dict("subregion" => Set([tregion])))
        #    remove_component_from_subsystem!(sys, fregion, fbus)
        #    add_component_to_subsystem!(sys, tregion, fbus)
        #end           
    end    
end

# EI
#=
include("EI_model.jl")
=#

#RTS
sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")

selected_line=["CA-1", "CB-1", "AB1"] 
ln="A28"
limit=1.0 #2.0

standardload=0
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

mipgap = 0.00001

#=
optimizer = optimizer_with_attributes(
    Gurobi.Optimizer,
    "Threads" => (length(Sys.cpu_info()) ÷ 2) - 1,
    "MIPGap" => mipgap,
    "TimeLimit" => 3000,
)
=#

ENV["XPRESSDIR"] = "C:\\xpressmp"
ENV["XPAUTH_PATH"] = "C:\\xpressmp\\bin"

optimizer = optimizer_with_attributes(
    Xpress.Optimizer,
    #"Threads" => (length(Sys.cpu_info()) ÷ 2) - 1,
    #"MIPGap" => mipgap,
    #"TimeLimit" => 3000,
)
#=
optimizer = optimizer_with_attributes(
    HiGHS.Optimizer,
)
=#
area_dict=Dict("1" => "1", "2" => "2", "3" =>"3")

CF="NS3-0"
NS=3
netmodels=["AreaPTDFPowerModel","AreaPTDFPowerModel","AreaPTDFPowerModel"]
copperplates=[1,0,0]
NTS=[24,24,1]
area_region_dicts=[Dict(),
                  Dict(),
                  Dict()]


CF="NS3-1"
NS=3
netmodels=["AreaPTDFPowerModel","SplitAreaPTDFPowerModel","SplitAreaPTDFPowerModel"]
copperplates=[1,0,0]
NTS=[24,24,1]
area_region_dicts=[Dict(),
                  Dict("1" => "a", "2" => "b", "3" =>"c"),
                  Dict("1" => "a", "2" => "b", "3" =>"c")]

monitoredline_subsys=[Dict(),Dict(ln => ["a"]),Dict(ln => ["a"])]
#=
CF="NS3-2"
NS=3
netmodels=["AreaPTDFPowerModel","SplitAreaPTDFPowerModel","SplitAreaPTDFPowerModel"]
copperplates=[1,0,0]
NTS=[24,24,1]
area_region_dicts=[Dict(),
                  Dict("1" => "a", "2" => "b", "3" =>"c"),
                  Dict("1" => "a", "2" => "b", "3" =>"c")]
monitoredline_subsys=[Dict(),Dict(ln => ["a","b","c"]),Dict(ln => ["a","b","c"])]
=#
#=
CF="NS3-3"
NS=3
netmodels=["AreaPTDFPowerModel","AreaPTDFPowerModel","SplitAreaPTDFPowerModel"]
copperplates=[1,0,0]
NTS=[24,24,1]
area_region_dicts=[Dict(),
                  Dict(),
                  Dict("1" => "a", "2" => "b", "3" =>"b")]
monitoredline_subsys=[Dict(),Dict(),Dict(ln => ["a","b"])]
=#
#=
NS=2
netmodels=["AreaPTDFPowerModel","SplitAreaPTDFPowerModel"]
copperplates=[1,0]
NTS=[24,24]
area_region_dicts=[Dict(),
                   Dict("1" => "a", "2" => "b", "3" =>"c")]
=#

syss=[sys]
templates=[]
ptdf = VirtualPTDF(sys; tol = 1e-4, max_cache_size = 10000)

l = PSY.get_component(ACBranch, sys, ln)
set_rating!(l,limit)
PSY.get_component(ACBranch, sys, ln)
          
monitoredlined_line=[ln]
          
for b in monitoredlined_line
    line = PSY.get_component(Line, sys, b)
    PSY.convert_component!(sys, line, MonitoredLine)
end

for i in 1:NS
    NT=NTS[i]
    if i>1                               
        sys2=deepcopy(sys)
        push!(syss,sys2)
    end
    transform_single_time_series!(syss[i], Hour(NT), Hour(NT))

    area_region_dict=area_region_dicts[i]  
    if netmodels[i]=="SplitAreaPTDFPowerModel"     
        subsys=unique(values(area_region_dict))
        for v in subsys
            println("add sys",i," subsystem,",v)
            add_subsystem!(sys2, v)
        end
        template = 
          MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true,PTDF_matrix = ptdf,), subsys)
    else          
        template = ProblemTemplate(NetworkModel(AreaPTDFPowerModel; use_slacks = true, PTDF_matrix = ptdf,))
    end

    push!(templates,template)    

    if standardload==1
        PSI.set_device_model!(template, StandardLoad , StaticPowerLoad)
    else
        PSI.set_device_model!(template, PowerLoad, StaticPowerLoad)
    end

    PSI.set_device_model!(template, RenewableDispatch, RenewableFullDispatch)
    PSI.set_device_model!(template, DeviceModel(HydroDispatch, HPS.HydroDispatchReservoirBudget,
                                    time_series_names = Dict{Any, String}(
                                        PSI.ActivePowerTimeSeriesParameter => "max_active_power",
                                        HPS.EnergyBudgetTimeSeriesParameter => "hydro_budget",)
                                            )
                    )
    #PSI.set_device_model!(template, TwoTerminalHVDCLine, HVDCTwoTerminalLossless)
    
    set_device_model!(template, DeviceModel(MonitoredLine, StaticBranchUnbounded, use_slacks = true))
    
    if NT>1              
        PSI.set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
        PSI.set_device_model!(template, ThermalMultiStart, ThermalBasicUnitCommitment)
        PSI.set_device_model!(template, AreaInterchange, StaticBranch)
    else
        PSI.set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
        PSI.set_device_model!(template, AreaInterchange, StaticBranchUnbounded)
    end                  
    
    if copperplates[i]==0
        set_device_model!(template, DeviceModel(MonitoredLine, StaticBranch, use_slacks = true))
    end
                  
    set_device_model!(template,
        DeviceModel(Line, StaticBranchUnbounded; attributes=Dict("filter_function" => x -> get_name(x) in selected_line),))             

    subsys=unique(values(area_region_dict))

    if length(subsys)>1
      #buildsubsystem1(sys2, area_region_dict, union(selected_line,monitoredlined_line))
      buildsubsystem1(sys2, area_region_dict, selected_line,monitoredline_subsys[i])
    end
end

#include("build_model_sim_run.jl")

dm=[]
for i in 1:NS
    if netmodels[i]=="SplitAreaPTDFPowerModel"
        push!(dm, DecisionModel(MultiRegionProblem, templates[i], syss[i], name="UC"*string(i),optimizer=optimizer,
            store_variable_names=true,initialize_model=false,optimizer_solve_log_print=false, 
            direct_mode_optimizer=true,check_numerical_bounds=false, 
            calculate_conflict=true,rebuild_model=false,system_to_file = true))
    else    
        push!(dm, DecisionModel(templates[i],syss[i], name="UC"*string(i), optimizer=optimizer,
        store_variable_names=true,initialize_model=false,optimizer_solve_log_print=false, 
        direct_mode_optimizer=true,check_numerical_bounds=false, 
        calculate_conflict=true,rebuild_model=false, system_to_file = true))
    end
end    

if NS==2
    models = PSI.SimulationModels(decision_models=[dm[1],dm[2]])
else    
    models = PSI.SimulationModels(decision_models=[dm[1],dm[2],dm[3]])
end    

uc_simulation_ffs=[]
feedforwards_dict=Dict()
for i in 1:NS-1
    uc_simulation_ff = Vector{PowerSimulations.AbstractAffectFeedforward}()
#feed forward area interchange
    FVFF_area_interchange = FixValueFeedforward(;component_type=AreaInterchange,source=FlowActivePowerVariable,affected_values=[FlowActivePowerVariable],)
    push!(uc_simulation_ff, FVFF_area_interchange)
    #feed forward HVDC (doesn't seem to work)
    #FVFF_hvdc = FixValueFeedforward(;component_type=TwoTerminalHVDCLine,source=FlowActivePowerVariable,affected_values=[FlowActivePowerVariable],)
    #push!(uc_simulation_ff, FVFF_hvdc)

    if NTS[i+1]==1
        SCFF = SemiContinuousFeedforward(;
        component_type=ThermalStandard, source=OnVariable, affected_values=[ActivePowerVariable],)
        push!(uc_simulation_ff, SCFF) 
        #LBFF = FixValueFeedforward(;component_type=ThermalStandard,source=OnVariable, affected_values=[OnVariable],)
        #push!(uc_simulation_ff, LBFF)
    end
    push!(uc_simulation_ffs,uc_simulation_ff)
    feedforwards_dict["UC"*string(i+1)]=uc_simulation_ff
end

sequence = SimulationSequence(;
    models=models,
    feedforwards=feedforwards_dict, #Dict("UC2" => uc_simulation_ff[1], "UC3" => uc_simulation_ff[2],),
    ini_cond_chronology=InterProblemChronology(),
);

# Specify the simulation setup
# Here we specify the simulation name, the initial/start time, number of steps/days to execute, and the simulation folder.
sim = PSI.Simulation(
    name="ntps_3stageDA_DASplit_RTSplit",
    steps=1,
    models=models,
    sequence=sequence,
    simulation_folder=mktempdir(), #".", #
)

println("before PSI.build")
PSI.build!(sim, serialize=false)
println("after PSI.build")
PSI.execute!(sim, enable_progress_bar=false)

results = SimulationResults(sim,ignore_status = true)

# check results and calculate flows in different optimization models and the actual flow called calculated_seflow
#include("final_check_results.jl")  
results_uc=[]; uc_areainterchange=[];uc_monitoredline=[];
bus_df_uc=[];area_df=[];areaflow_df=[]
tm=Array(1:NTS[1])
af=DataFrames.DataFrame(;tm)

for i in 1:NS
    push!(results_uc,get_decision_problem_results(results, "UC"*string(i)))
    push!(uc_areainterchange,read_realized_variable(results_uc[i], "FlowActivePowerVariable__AreaInterchange"))
    push!(uc_monitoredline,read_realized_variable(results_uc[i], "FlowActivePowerVariable__MonitoredLine"))

    if netmodels[i]=="SplitAreaPTDFPowerModel"
        push!(bus_df_uc,read_bus_df(results_uc[i], ln,1,standardload)) 
    else    
        push!(bus_df_uc,read_bus_df(results_uc[i], ln,0,standardload))
    end    

    bus_df_uc[i][!,"buscheck"]=bus_df_uc[i][!,"ActivePowerBalance__ACBus"]-bus_df_uc[i][!,"HVDC"]/100-bus_df_uc[i][!,"ThermalStandard"]/100-bus_df_uc[i][!,"Renewable"]/100-bus_df_uc[i][!,"Hydro"]/100-bus_df_uc[i][!,"PowerLoad"]/100
    println("UC0 bus check ActivePowerBalance__ACBus<>Gen/100-PowerLoad/100 ,",filter([:t,:buscheck] => (t,buscheck)-> (t>=1) && (abs(buscheck)>0.000001),  bus_df_uc[i]))
    # if some bus has non-zero buscheck value, then check the bus. Some MW may not be added to power balance correctly.
    #println(filter([:t,:bus] => (t,bus)-> (t==1) && (bus==113 || bus==316),  bus_df_uc0))

###### area MW check #########
#area level net Gen, load and ActivePowerBalance__ACBus_sum check
    push!(area_df,combine(groupby(bus_df_uc[i], [:t, :area]), [:ThermalStandard, :Renewable, :Hydro, :PowerLoad, :ActivePowerBalance__ACBus, :StateEstimationInjections__ACBus] .=> sum))
    area_df[i][!,"areacheck"]=area_df[i][!,"ActivePowerBalance__ACBus_sum"]-area_df[i][!,"ThermalStandard_sum"]/100-area_df[i][!,"Renewable_sum"]/100-area_df[i][!,"Hydro_sum"]/100-area_df[i][!,"PowerLoad_sum"]/100
    println("i,area_df check,",sum(area_df[i][!,"areacheck"]))  #should be close to 0

    area_df[i][!,"TotalGen"]=area_df[i][!,"ThermalStandard_sum"]+area_df[i][!,"Renewable_sum"]+area_df[i][!,"Hydro_sum"]
    area_df[i][!,"NetInterchange"]=area_df[i][!,"TotalGen"]+area_df[i][!,"PowerLoad_sum"]
    area_df[i][!,"InterchangeCheck"]=area_df[i][!,"NetInterchange"]-100*area_df[i][!,"ActivePowerBalance__ACBus_sum"]
    println(area_df[i][:,["t","area","ActivePowerBalance__ACBus_sum","NetInterchange","InterchangeCheck"]])

####### area flow check #########
#area flow contribution by area and interval
    push!(areaflow_df, combine(groupby(bus_df_uc[i], [:t, :area]), [:flowcontribution, :loopflowcontribution] .=> sum))

    if length(area_region_dicts[i])==0
        #j=findfirst(!=(0), length.(area_region_dicts))
        #area_region_dict=area_region_dicts[j] 
        area_region_dict=area_dict
    else   
        area_region_dict=area_region_dicts[i]
    end      
    
    subsys=unique(values(area_region_dict))

    calculated_flow_subsys=Dict()
    calculated_loopflow_subsys=Dict()
    NT=NTS[1]
    for s in subsys
        #calculated_flow_subsys[s]=zeros(NT)
        #calculated_loopflow_subsys[s]=zeros(NT)
        af[!,"uc"*string(i)*"_flow_subsys_"*s]=zeros(NT)
        af[!,"uc"*string(i)*"_loopflow_subsys_"*s]=zeros(NT)
    end    

    if netmodels[i]!="SplitAreaPTDFPowerModel"
        for r in keys(area_region_dict)
            s=area_region_dict[r]
            name="uc"*string(i)*"_flow_subsys_"*s
            af[!,name]=af[!,name]+filter([:t,:area]=>(t,area)->(area==r),areaflow_df[i])[!,:flowcontribution_sum]
            #calculated_flow_subsys[s]=calculated_flow_subsys[s]+ filter([:t,:area]=>(t,area)->(area==r),areaflow_df[i])[!,:flowcontribution_sum]
        end
        #calculated_flow_uc0=zeros(NT)
        #for s in subsys
        #    calculated_flow_uc0=calculated_flow_uc0+calculated_flow_uc0_subsys[s]
        #end  
        af[!,"uc"*string(i)*"_flow"]= zeros(NT) #calculated_flow_uc0

        #for (k,v) in calculated_flow_uc0_subsys
        #    name="uc"*string(i)*"_flow_subsys"*k
        #    af[!,name]=calculated_flow_uc0_subsys[k]
        for s in subsys
            af[!,"uc"*string(i)*"_flow"]=af[!,"uc"*string(i)*"_flow"]+af[!,"uc"*string(i)*"_flow_subsys_"*s]#calculated_flow_uc0_subsys[k]
        end 
    else
        #for r in keys(area_region_dict)
        #    s=area_region_dict[r]
        #    calculated_flow_subsys[s]=calculated_flow_subsys[s]+ filter([:t,:area]=>(t,area)->(area==r),areaflow_df[i])[!,:flowcontribution_sum]
        #    calculated_loopflow_subsys[s]=calculated_loopflow_subsys[s]+ filter([:t,:area]=>(t,area)->(area==r),areaflow_df[i])[!,:loopflowcontribution_sum]
        #end

        for r in keys(area_region_dict)
            s=area_region_dict[r]
            name="uc"*string(i)*"_flow_subsys_"*s
            af[!,name]=af[!,name]+filter([:t,:area]=>(t,area)->(area==r),areaflow_df[i])[!,:flowcontribution_sum]
            #calculated_flow_subsys[s]=calculated_flow_subsys[s]+ filter([:t,:area]=>(t,area)->(area==r),areaflow_df[i])[!,:flowcontribution_sum]
            name="uc"*string(i)*"_loopflow_subsys_"*s
            af[!,name]=af[!,name]+filter([:t,:area]=>(t,area)->(area==r),areaflow_df[i])[!,:loopflowcontribution_sum]
        end

        calculated_flow_uc2=Dict()
        #calculated_seflow_uc2=zeros(NT)
        af[!,"uc"*string(i)*"_seflow"]=zeros(NT)
        for s in subsys
            #calculated_flow_uc2[s]=zeros(NT)
            #calculated_seflow_uc2=calculated_seflow_uc2+calculated_flow_subsys[s]
            af[!,"uc"*string(i)*"_flow_"*s]=zeros(NT)
            af[!,"uc"*string(i)*"_seflow"]=af[!,"uc"*string(i)*"_seflow"]+af[!,"uc"*string(i)*"_flow_subsys_"*s]#calculated_flow_subsys[s]
            for s1 in subsys
                if s==s1
                    #calculated_flow_uc2[s]=calculated_flow_uc2[s]+calculated_flow_subsys[s1]
                    name="uc"*string(i)*"_flow_"*s
                    af[!,name]=af[!,name]+af[!,"uc"*string(i)*"_flow_subsys_"*s1]#calculated_flow_subsys[s1]
                else
                    #calculated_flow_uc2[s]=calculated_flow_uc2[s]+calculated_loopflow_subsys[s1]  
                    name="uc"*string(i)*"_flow_"*s
                    af[!,name]=af[!,name]+af[!,"uc"*string(i)*"_loopflow_subsys_"*s1]#calculated_loopflow_subsys[s1]  
                end    
            end  
        end
    end
end        


CSV.write("af_"*CF*".csv",af)

#=
uc0=sim.models.decision_models[1].internal.container
uc2=sim.models.decision_models[2].internal.container
sum(value.(uc0.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:])-af[!,"uc1_flow"])
sum(value.(uc2.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:])-af[!,"uc2_flow"])

uc2a=sim.models.decision_models[2].internal.container.subproblems["a"]
uc2b=sim.models.decision_models[2].internal.container.subproblems["b"]
uc2c=sim.models.decision_models[2].internal.container.subproblems["c"]
sum(value.(uc0.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:])-af[!,"uc1_flow"])
sum(value.(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:])-af[!,"uc2_flow_a"])
sum(value.(uc2b.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:])-af[!,"uc2_flow_b"])
sum(value.(uc2c.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:])-af[!,"uc2_flow_c"])

uc3a=sim.models.decision_models[3].internal.container.subproblems["a"]
uc3b=sim.models.decision_models[3].internal.container.subproblems["b"]
sum(value.(uc3a.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:])-af[!,"uc3_flow_a"])
sum(value.(uc3b.variables[InfrastructureSystems.Optimization.VariableKey{FlowActivePowerVariable, MonitoredLine}("")]["A28",:])-af[!,"uc3_flow_b"])

uc2a.constraints[InfrastructureSystems.Optimization.ConstraintKey{NetworkFlowConstraint, Line}("")] 
uc2a.constraints[InfrastructureSystems.Optimization.ConstraintKey{RateLimitConstraint, MonitoredLine}("")]

uc2a.constraints[InfrastructureSystems.Optimization.ConstraintKey{CopperPlateBalanceConstraint, Area}("")] 

objective_value(uc2a.JuMPmodel)+objective_value(uc2b.JuMPmodel)+objective_value(uc2c.JuMPmodel)-objective_value(uc0.JuMPmodel)

objterms(uc2,"obj_uc2a_"*CF)
write_coeff(uc2,"A28",1,"A28_uc2a_"*CF,1)

objterms(uc2a,"obj_uc2a_fix_"*CF)
write_coeff(uc2a,"A28",1,"A28_uc2a_fix_"*CF,1)


fix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,RenewableDispatch}("")]["122_WIND_1",1],1.8596521707224; force=true)
fix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["107_CC_1",1],1.71962875337936; force=true)
fix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["123_STEAM_3",1],2.14337921320769; force=true)
fix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["123_CT_5",1],0.22; force=true)
fix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["121_NUCLEAR_1",1],4; force=true)

fix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["115_STEAM_3",1],0; force=true)
fix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["116_STEAM_1",1],0; force=true)
fix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["118_CC_1",1],0; force=true)

fix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["123_CT_1",1],0; force=true)
fix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["123_CT_4",1],0; force=true)

unfix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["115_STEAM_3",1])
unfix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["116_STEAM_1",1])
unfix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["118_CC_1",1])
unfix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["123_CT_1",1])
unfix(uc2a.variables[InfrastructureSystems.Optimization.VariableKey{ActivePowerVariable,ThermalStandard}("")]["123_CT_4",1])


con2a=uc2a.constraints[InfrastructureSystems.Optimization.ConstraintKey{CopperPlateBalanceConstraint, Area}("")]["1",1]
write_cons_coeff(uc2a,con2a,"pb_1_1_fix_"*CF)

PSI.compute_conflict!(uc2a)

con2=uc2.constraints[InfrastructureSystems.Optimization.ConstraintKey{CopperPlateBalanceConstraint, Area}("")]["1",1]
write_cons_coeff(uc2,con2,"pb_1_1_"*CF)


=#