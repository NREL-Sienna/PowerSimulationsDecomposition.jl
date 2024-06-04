##### Methods for SplitAreaPTDFPowerModel #######
# This method generates the correct expressions that include the other Areas' injections
# as parameters

PSI._system_expression_type(::Type{SplitAreaPTDFPowerModel}) = PSY.Area

function PSI._ref_index(::PSI.NetworkModel{SplitAreaPTDFPowerModel}, device_bus::PSY.ACBus)
    return PSY.get_name(PSY.get_area(device_bus))
end

function PSI.construct_network!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
    ::PSI.ProblemTemplate,
)
    if PSI.get_use_slacks(model)
        PSI.add_variables!(container, PSI.SystemBalanceSlackUp, sys, model)
        PSI.add_variables!(container, PSI.SystemBalanceSlackDown, sys, model)
        PSI.add_to_expression!(container, PSI.ActivePowerBalance, PSI.SystemBalanceSlackUp, sys, model)
        PSI.add_to_expression!(
            container,
            PSI.ActivePowerBalance,
            PSI.SystemBalanceSlackDown,
            sys,
            model,
        )
        PSI.objective_function!(container, sys, model)
    end
    PSI.add_parameters!(container, StateEstimationInjections, sys, model)
    PSI.add_constraints!(container, PSI.CopperPlateBalanceConstraint, sys, model)
    PSI.add_constraints!(container, PSI.NodalBalanceActiveConstraint, sys, model)
    PSI.add_constraint_dual!(container, sys, model)
    return
end

function PSI.objective_function!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
)
    variable_up = PSI.get_variable(container, PSI.SystemBalanceSlackUp(), PSY.Area)
    variable_dn = PSI.get_variable(container, PSI.SystemBalanceSlackDown(), PSY.Area)
    areas = PSY.get_name.(PSI.get_available_components(network_model, PSY.Area, sys))

    for t in PSI.get_time_steps(container), n in areas
        PSI.add_to_objective_invariant_expression!(
            container,
            (variable_dn[n, t] + variable_up[n, t]) * PSI.BALANCE_SLACK_COST,
        )
    end
    return
end

function PSI.add_parameters!(
    container::OptimizationContainer,
    T::Type{StateEstimationInjections},
    sys::PSY.System,
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
)
    if get_rebuild_model(get_settings(container)) && has_container_key(container, T, D)
        return
    end



    return
end

function PSI.initialize_system_expressions!(
    container::PSI.OptimizationContainer,
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
    subnetworks::Dict{Int, Set{Int}},
    system::PSY.System,
    ::Dict{Int64, Set{Int64}},
)
    areas = PSI.get_available_components(network_model, PSY.Area, system)
    @assert !isempty(areas)
    dc_bus_numbers = [
        PSY.get_number(b) for
        b in PSI.get_available_components(network_model, PSY.DCBus, system)
    ]
    PSI._make_system_expressions!(
        container,
        subnetworks,
        dc_bus_numbers,
        PSI.AreaPTDFPowerModel,
        areas,
    )
    return
end

function PSI.add_to_expression!(
    container::PSI.OptimizationContainer,
    ::Type{T},
    ::Type{U},
    devices::IS.FlattenIteratorWrapper{V},
    device_model::PSI.DeviceModel{V, W},
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
) where {
    T <: PSI.ActivePowerBalance,
    U <: PSI.VariableType,
    V <: PSY.StaticInjection,
    W <: PSI.AbstractDeviceFormulation,
}
    variable = PSI.get_variable(container, U(), V)
    area_expr = PSI.get_expression(container, T(), PSY.Area)
    nodal_expr = PSI.get_expression(container, T(), PSY.ACBus)
    radial_network_reduction = PSI.get_radial_network_reduction(network_model)
    for d in devices
        name = PSY.get_name(d)
        device_bus = PSY.get_bus(d)
        area_name = PSY.get_name(PSY.get_area(device_bus))
        bus_no = PNM.get_mapped_bus_number(radial_network_reduction, device_bus)
        for t in PSI.get_time_steps(container)
            PSI._add_to_jump_expression!(
                area_expr[area_name, t],
                variable[name, t],
                PSI.get_variable_multiplier(U(), V, W()),
            )
            PSI._add_to_jump_expression!(
                nodal_expr[bus_no, t],
                variable[name, t],
                PSI.get_variable_multiplier(U(), V, W()),
            )
        end
    end
    return
end

function PSI.add_to_expression!(
    container::PSI.OptimizationContainer,
    ::Type{T},
    ::Type{U},
    sys::PSY.System,
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
) where {
    T <: PSI.ActivePowerBalance,
    U <: Union{PSI.SystemBalanceSlackUp, PSI.SystemBalanceSlackDown},
}
    variable = PSI.get_variable(
        container,
        U(),
        PSI._system_expression_type(SplitAreaPTDFPowerModel),
    )
    expression = PSI.get_expression(
        container,
        T(),
        PSI._system_expression_type(SplitAreaPTDFPowerModel),
    )
    areas = PSI.get_available_components(network_model, PSY.Area, sys)
    for t in PSI.get_time_steps(container), n in PSY.get_name.(areas)
        PSI._add_to_jump_expression!(
            expression[n, t],
            variable[n, t],
            PSI.get_variable_multiplier(U(), PSY.Area, SplitAreaPTDFPowerModel()),
        )
    end
    return
end

function PSI.add_variables!(
    container::PSI.OptimizationContainer,
    ::Type{T},
    sys::PSY.System,
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
) where {
    T <: Union{PSI.SystemBalanceSlackUp, PSI.SystemBalanceSlackDown},
}
    time_steps = PSI.get_time_steps(container)
    areas = PSY.get_name.(PSI.get_available_components(network_model, PSY.Area, sys))
    variable =
        PSI.add_variable_container!(container, T(), PSY.Area, areas, time_steps)

    for t in time_steps, area in areas
        variable[area, t] = JuMP.@variable(
            PSI.get_jump_model(container),
            base_name = "slack_{$(T), $(area), $t}",
            lower_bound = 0.0
        )
    end

    return
end
