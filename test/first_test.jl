# build function able to exclude elements belonging to a certain sub region
# THIS IS A HACK!!!

# TODO: add field `ext` to all the topology types (Area, LoadZone, Arc)

# SIIP Packages
using Revise

using HydroPowerSimulations
using PowerSimulations
using PowerSystemCaseBuilder
using InfrastructureSystems
using StorageSystemsSimulations
using PowerSystems
using Xpress
using PowerSimulationsDecomposition
using Logging
# using Plots

# using Xpress
# using JuMP
# using Logging
# using Dates
# using DataFrames

const PSY = PowerSystems
const PSI = PowerSimulations
const PSB = PowerSystemCaseBuilder

# consider the use of custom system used for GDO case
name_ = "AC_inter"
sys_twin_rts_DA = PSY.System("GDO systems/saved_main_RTS_GMLC_DA_final_sys_" * name_ * ".json")   # day ahead 

# ! check reserves


# modify the system -> add features in the "ext" field
for d in PSY.get_components(PSY.Component, sys_twin_rts_DA)
    if typeof(d) <: PSY.Bus || :available in fieldnames(typeof(d))
        if occursin("twin", PSY.get_name(d))
            PSY.set_ext!(d, Dict("subregion" => Set(["2"])))
        else
            PSY.set_ext!(d, Dict("subregion" => Set(["1"])))
        end
    end
end

# interconnection is shared between the two regions
if name_ == "AC_inter"
    br = get_component(PSY.MonitoredLine, sys_twin_rts_DA, "AC_interconnection")
elseif name_ == "HVDC_inter"
    br = get_component(PSY.TwoTerminalHVDCLine, sys_twin_rts_DA, "HVDC_interconnection")
end

PSY.set_ext!(br, Dict("subregion" => Set(["1", "2"])))
arc_ = get_arc(br)
for b in [get_from(arc_), get_to(arc_)]
    PSY.set_ext!(b, Dict("subregion" => Set(["1", "2"])))
end


if name_ == "HVDC_inter"
    HVDC_inter = true
else
    HVDC_inter = false
end

# DC power flow reference solution

# define battery model
storage_model = DeviceModel(
    GenericBattery,
    StorageDispatchWithReserves;
    attributes=Dict(
        "reservation" => false,
        "cycling_limits" => false,
        "energy_target" => false,
        "complete_coverage" => false,
        "regularization" => true
    ),
)

# UC model
template_uc =
    ProblemTemplate(
        # NetworkModel(StandardPTDFModel; PTDF_matrix = PTDF(sys_twin_rts)),
        NetworkModel(DCPPowerModel; use_slacks=true),
    )
set_device_model!(template_uc, ThermalStandard, ThermalStandardUnitCommitment)
set_device_model!(template_uc, RenewableDispatch, RenewableFullDispatch)
set_device_model!(template_uc, RenewableFix, FixedOutput)
set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
set_device_model!(template_uc, Line, StaticBranch)
set_device_model!(template_uc, Transformer2W, StaticBranchUnbounded)
set_device_model!(template_uc, TapTransformer, StaticBranchUnbounded)
set_device_model!(template_uc, HydroDispatch, FixedOutput)
set_device_model!(template_uc, HydroEnergyReservoir, FixedOutput)
set_device_model!(template_uc, storage_model)
set_service_model!(
    template_uc,
    ServiceModel(VariableReserve{ReserveUp}, RangeReserve; use_slacks = true),
)
set_service_model!(
    template_uc,
    ServiceModel(VariableReserve{ReserveDown}, RangeReserve; use_slacks = true),
)

# add the HVDC line in case is present
if HVDC_inter == "true"
    set_device_model!(template_uc, TwoTerminalHVDCLine, HVDCTwoTerminalDispatch)
else
    set_device_model!(template_uc, MonitoredLine, StaticBranch)
end

model = DecisionModel(
    MultiRegionProblem,
    template_uc,
    sys_twin_rts_DA;
    name = "UC",
    optimizer = optimizer_with_attributes(
        Xpress.Optimizer, 
        "MIPRELSTOP" => 0.01,       # Set the relative mip gap tolerance
        "MAXMEMORYSOFT" => 600000,   # Set the maximum amount of memory the solver can use (in MB)
    ),
    system_to_file = false,
    initialize_model = true,
    optimizer_solve_log_print = true,
    direct_mode_optimizer = true,
    rebuild_model = false,
    store_variable_names = true,
    calculate_conflict = true,
)

for b in get_components(ACBus, model.sys)
    @show get_bustype(b)
    @show get_bustype(b) == PSY.ACBusTypes.ISOLATED
    # if get_bustype(b) == PSY.ACBusTypes.ISOLATED
    #     @show get_name(b)
    # end
end

# b = get_component(ACBus, model.sys, "Caesar")
# get_bustype(b) == ACBusTypes.ISOLATED
# get_name(b)

# PowerSimulationsDecomposition.instantiate_network_model(model)

build!(model; console_level = Logging.Info, output_dir = mktempdir())
solve!(model; console_level = Logging.Info)