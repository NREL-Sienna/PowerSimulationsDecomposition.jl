##### construct_device! overrides for InterconnectingConverter on SplitAreaPTDFPowerModel #######
# The upstream PSI construct_device! for InterconnectingConverter dispatches on
# NetworkModel{<:PM.AbstractActivePowerModel} and calls add_to_expression! without sys.
# PSD's add_to_expression! specialization for SplitAreaPTDFPowerModel requires sys to
# filter converters to only those belonging to the current subproblem's subsystem.
# These methods dispatch more specifically (SplitAreaPTDFPowerModel vs the generic
# AbstractActivePowerModel), so Julia picks them first and forwards sys correctly.

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.InterconnectingConverter, PSI.LosslessConverter},
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, PSI.LosslessConverter())
    PSI.add_to_expression!(
        container,
        sys,
        PSI.ActivePowerBalance,
        PSI.ActivePowerVariable,
        devices,
        model,
        network_model,
    )
    PSI.add_feedforward_arguments!(container, model, devices)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{PSY.InterconnectingConverter, PSI.LosslessConverter},
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_feedforward_constraints!(container, model, devices)
    PSI.objective_function!(
        container,
        devices,
        model,
        PSI.get_network_formulation(network_model),
    )
    PSI.add_constraint_dual!(container, sys, model)
    return
end
