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
    PSI.add_to_expression!(
            container,
            PSI.ActivePowerBalance,
            StateEstimationInjections,
            sys,
            model,
        )
    PSI.add_constraints!(container, PSI.CopperPlateBalanceConstraint, sys, model)
    PSI.add_constraints!(container, PSI.NodalBalanceActiveConstraint, sys, model)
    PSI.add_constraint_dual!(container, sys, model)
    return
end

"""
uses the area balance for areas inside of the subsystem
"""
function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{T},
    sys::U,
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
) where {
    T <: PSI.CopperPlateBalanceConstraint,
    U <: PSY.System,
}
    time_steps = PSI.get_time_steps(container)
    expressions = PSI.get_expression(container, PSI.ActivePowerBalance(), PSY.Area)
    area_names = PSY.get_name.(PSI.get_available_components(network_model, PSY.Area, sys))
    constraint =
    PSI.add_constraints_container!(container, T(), PSY.Area, area_names, time_steps)
    jm = PSI.get_jump_model(container)
    for t in time_steps, k in area_names
        constraint[k, t] = JuMP.@constraint(jm, expressions[k, t] == 0)
    end

    return
end

function PSI.add_to_expression!(
    container::PSI.OptimizationContainer,
    ::Type{PSI.ActivePowerBalance},
    ::Type{StateEstimationInjections},
    sys::PSY.System,
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
)
    parameter_array = PSI.get_parameter_array(container, StateEstimationInjections(), PSY.ACBus)
    subsys= PSI.get_subsystem(network_model)
    all_buses = PSY.get_components(
        x -> PSY.get_bustype(x) != PSY.ACBusTypes.ISOLATED,
        PSY.ACBus,
        sys;
    )

    # These are the buses not in the same subsystem as the one being built

    expression = PSI.get_expression(container, PSI.ActivePowerBalance(), PSY.ACBus)
    radial_network_reduction = PSI.get_radial_network_reduction(network_model)
    for b in all_buses, t in PSI.get_time_steps(container)
        if PSY.has_component(sys, subsys, b)
            continue
        end
        bus_no = PNM.get_mapped_bus_number(radial_network_reduction, b)
        PSI._add_to_jump_expression!(
            expression[bus_no, t],
            parameter_array[string(bus_no), t],
            1.0,
        )
    end
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
    container::PSI.OptimizationContainer,
    ::Type{StateEstimationInjections},
    sys::PSY.System,
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
)

    time_steps = PSI.get_time_steps(container)
    subsys= PSI.get_subsystem(network_model)

    all_buses = PSY.get_components(
        x -> PSY.get_bustype(x) != PSY.ACBusTypes.ISOLATED,
        PSY.ACBus,
        sys;
    )

    # These are the buses not in the same subsystem as the one being built
    bus_numbers = [string(PSY.get_number(b)) for b in all_buses if !PSY.has_component(sys, subsys, b)]
    @assert !isempty(bus_numbers)

    parameter_container = PSI.add_param_container!(
        container,
        StateEstimationInjections(),
        PSY.ACBus,
        ISOPT.ExpressionKey{PSI.ActivePowerBalance, PSY.ACBus}(""),
        bus_numbers,
        time_steps)

    time_steps = PSI.get_time_steps(container)
    jump_model = PSI.get_jump_model(container)

    for b_no in bus_numbers, t in time_steps
        PSI.set_multiplier!(
                parameter_container,
                1.0,
                b_no,
                t,
            )
        PSI.set_parameter!(
                parameter_container,
                jump_model,
                0.0,
                b_no,
                t,
            )
    end
    return
end

function PSI.initialize_system_expressions!(
    container::PSI.OptimizationContainer,
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
    subnetworks::Dict{Int, Set{Int}},
    system::PSY.System,
    bus_reduction_map::Dict{Int64, Set{Int64}},
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
        bus_reduction_map
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

function _update_parameter_values!(
    parameter_array::AbstractArray{T},
    attributes::PSI.VariableValueAttributes,
    model::PSI.DecisionModel{MultiRegionProblem},
    state::PSI.DatasetContainer{PSI.InMemoryDataset},
) where {T <: Union{JuMP.VariableRef, Float64}}
    current_time = PSI.get_current_time(model)
    state_values = PSI.get_dataset_values(state, PSI.get_attribute_key(attributes))
    component_names, time = axes(parameter_array)
    model_resolution = PSI.get_resolution(model)
    state_data = PSI.get_dataset(state, PSI.get_attribute_key(attributes))
    state_timestamps = state_data.timestamps
    max_state_index = PSI.get_num_rows(state_data)
    if model_resolution < state_data.resolution
        t_step = 1
    else
        t_step = model_resolution ÷ state_data.resolution
    end
    state_data_index = PSI.find_timestamp_index(state_timestamps, current_time)
    sim_timestamps = range(current_time; step = model_resolution, length = time[end])
    for t in time
        timestamp_ix = min(max_state_index, state_data_index + t_step)
        @debug "parameter horizon is over the step" max_state_index > state_data_index + 1
        if state_timestamps[timestamp_ix] <= sim_timestamps[t]
            state_data_index = timestamp_ix
        end
        for name_ix in component_names
            # Pass indices in this way since JuMP DenseAxisArray don't support view()
            state_value = state_values[name_ix, state_data_index]
            if !isfinite(state_value)
                error(
                    "The value for the system state used in $(encode_key_as_string(get_attribute_key(attributes))) is not a finite value $(state_value) \
                     This is commonly caused by referencing a state value at a time when such decision hasn't been made. \
                     Consider reviewing your models' horizon and interval definitions",
                )
            end
            PSI._set_param_value!(parameter_array, state_value, name_ix, t)
        end
    end
    return
end

"""
Update parameter function an OperationModel
"""
function PSI.update_container_parameter_values!(
    optimization_container::PSI.OptimizationContainer,
    model::PSI.DecisionModel{MultiRegionProblem},
    key::PSI.ParameterKey{StateEstimationInjections, PSY.ACBus},
    input::PSI.DatasetContainer{PSI.InMemoryDataset},
)
    # Enable again for detailed debugging
    # TimerOutputs.@timeit RUN_SIMULATION_TIMER "$T $U Parameter Update" begin
    # Note: Do not instantite a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    parameter_array = PSI.get_parameter_array(optimization_container, key)
    parameter_attributes = PSI.get_parameter_attributes(optimization_container, key)
    _update_parameter_values!(parameter_array, parameter_attributes, model, input)
    return
end
