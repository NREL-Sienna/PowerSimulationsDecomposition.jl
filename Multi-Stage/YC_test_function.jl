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

function read_bus_df(results_rt,line,se=0)
    ac = read_realized_variable(results_rt, "ActivePowerVariable__ThermalStandard")
    ab = read_realized_variable(results_rt, "ActivePowerBalance__ACBus")
    if se==1  al = read_realized_variable(results_rt, "StateEstimationInjections__ACBus") end
    ald = read_realized_variable(results_rt, "ActivePowerTimeSeriesParameter__PowerLoad")
    am = read_realized_variable(results_rt, "FlowActivePowerVariable__MonitoredLine")

    bus_ActivePowerVariable__ThermalStandard=Dict()
    bus_ActivePowerTimeSeriesParameter__PowerLoad=Dict()
    bus_ActivePowerBalance__ACBus=Dict()
    bus_StateEstimationInjections__ACBus=Dict()

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

function write_param_acbus(name,uc2a,line)
    parameter_ACBus_2a=uc2a.parameters[InfrastructureSystems.Optimization.ParameterKey{PowerSimulationsDecomposition.StateEstimationInjections, ACBus}("")]
    i=axes(parameter_ACBus_2a.parameter_array)
    for j2 in i[2]
      for j1 in i[1]
        con=uc2a.constraints[InfrastructureSystems.Optimization.ConstraintKey{NetworkFlowConstraint, MonitoredLine}("")][line,j2] 
        param=parameter_ACBus_2a.parameter_array[j1,j2]
        println(name,",j1,j2,",j1,",",j2,",value,",value(parameter_ACBus_2a.parameter_array[j1,j2]),",coeff,",normalized_coefficient(con,param))
       end 
    end    
end 

#PSI.get_expression(uc0, ActivePowerBalance(), PSY.ACBus)[101,2].terms

function write_coeff(uc2a,linename,t,fnm,use_monitoredline)
    fixedmw=0; fixedflow=0; areamw=Dict(); areaflow=Dict(); areaload=Dict(); arealoadflow=Dict()
    for j=1:3  areamw[j]=0; areaflow[j]=0  end
    c=uc2a.constraints[InfrastructureSystems.Optimization.ConstraintKey{CopperPlateBalanceConstraint, Area}("")][:,t]
    for i in axes(c)[1]
        for k in all_variables(uc2a.JuMPmodel)  
            if normalized_coefficient(c[i], k)!=0         
                #println("coef;",name(k),";",normalized_coefficient(con2a, k), ";is_fixed;",is_fixed(k),";value;",value(k))
                if name(k)=="param" 
                   areaload[i]=-normalized_coefficient(c[i], k)
                   println("area,",i,",",areaload[i])
                end
            end       
        end
    end               

    if use_monitoredline==1
        con2a=uc2a.constraints[InfrastructureSystems.Optimization.ConstraintKey{NetworkFlowConstraint, MonitoredLine}("")][linename,t]
    else    
        con2a=uc2a.constraints[InfrastructureSystems.Optimization.ConstraintKey{NetworkFlowConstraint, Line}("")][linename,t]
    end    
    open(fnm,"w") do io
        redirect_stdout(io) do
            for k in all_variables(uc2a.JuMPmodel)  
                if normalized_coefficient(con2a, k)!=0         
                    println("coef;",name(k),";",normalized_coefficient(con2a, k), ";is_fixed;",is_fixed(k),";value;",value(k))
                    if name(k)=="param" 
                        fixedmw=fixedmw+value(k); 
                        fixedflow=fixedflow+value(k)*normalized_coefficient(con2a, k) 
                    else
                        str=split(name(k),"{");
                        println("name,",name(k),",str,",str)
                        if first(str[1],1)=="A"
                            a= parse(Int64,first(str[2],1));# tt=firstindex(split(str[2],", "))
                            areamw[a]=areamw[a]+value(k)
                            areaflow[a]=areaflow[a]+value(k)*normalized_coefficient(con2a, k) 
                            #println("a,areamw,areaflow,",a,",",areamw[a],",",areaflow[a])
                        end    
                    end
                end  
            end 
        end
    end
    return(fixedmw,areamw,fixedflow,areaflow)      
end

function write_param_internalload(name,uc0,line)
    parameter_load_0=uc0.parameters[InfrastructureSystems.Optimization.ParameterKey{ActivePowerTimeSeriesParameter, PowerLoad}("")]
    i=axes(parameter_load_0.parameter_array)
    for j2 in i[2]
       for j1 in i[1]
        con=uc0.constraints[InfrastructureSystems.Optimization.ConstraintKey{NetworkFlowConstraint, MonitoredLine}("")][line,j2] 
        param=parameter_load_0.parameter_array[j1,j2]
        println(name,",load,j1,j2,",j1,",",j2,",value,",value(parameter_load_0.parameter_array[j1,j2]),",coeff,",normalized_coefficient(con,param))
       end 
    end    
end    
    
function write_acbus(ucname,uc0,ax,t, debug=0)
    area_param=Dict(); area_gen=Dict()
    area_param[1]=0;area_param[2]=0;area_param[3]=0
    area_gen[1]=0;area_gen[2]=0;area_gen[3]=0 
    for bus in ax[1] 
        for k in keys(PSI.get_expression(uc0, ActivePowerBalance(), PSY.ACBus)[bus,t].terms)
            coef=PSI.get_expression(uc0, ActivePowerBalance(),PSY.ACBus)[bus,t].terms[k]
            v=value(k); 
            if debug==1
                println(ucname,";bus;t;",bus,";",t,";",k,";coefficient;", coef,";solution;",v,";coef*v;",coef*v)
            end    
            n=Int64(floor(bus/100))
            if name(k)=="param"  area_param[n]=area_param[n]+coef*v else area_gen[n]=area_gen[n]+coef*v end
        end
    end    
    return(area_gen,area_param)
end

function buildsubsystem(sys, enforced_region, selected_line)
    add_subsystem!(sys, "a")
    add_subsystem!(sys, "b")

    for b in PSY.get_components(PSY.Area, sys)
        if get_name(b) == enforced_region[1] || get_name(b) == enforced_region[2]
            PSY.set_ext!(b, Dict("subregion" => Set(["a"])))
            add_component_to_subsystem!(sys, "a", b)
        else
            PSY.set_ext!(b, Dict("subregion" => Set(["b"])))
            add_component_to_subsystem!(sys, "b", b)
        end
    end
    for b in PSY.get_components(PSY.StaticInjection, sys)
        if get_name(get_area(get_bus(b))) == enforced_region[1] || get_name(get_area(get_bus(b))) == enforced_region[2]
            PSY.set_ext!(b, Dict("subregion" => Set(["a"])))
            add_component_to_subsystem!(sys, "a", b)
        else
            PSY.set_ext!(b, Dict("subregion" => Set(["b"])))
            add_component_to_subsystem!(sys, "b", b)
        end
    end
    for b in PSY.get_components(PSY.Bus, sys)
        if get_name(get_area(b)) == enforced_region[1] || get_name(get_area(b)) == enforced_region[2]
            PSY.set_ext!(b, Dict("subregion" => Set(["a"])))
            add_component_to_subsystem!(sys, "a", b)
        else
            PSY.set_ext!(b, Dict("subregion" => Set(["b"])))
            add_component_to_subsystem!(sys, "b", b)
        end
    end
    for b in selected_line 
        l=PSY.get_component(ACBranch, sys,b)
        PSY.set_ext!(l, Dict("subregion" => Set(["a"])))
        add_component_to_subsystem!(sys, "a", l)
        PSY.set_ext!(l, Dict("subregion" => Set(["b"])))
        add_component_to_subsystem!(sys, "b", l)  
    end                  
end