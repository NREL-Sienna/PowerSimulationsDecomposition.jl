function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.MonitoredLine, StaticBranchUnboundedStateEstimation},
    network_model::PSI.NetworkModel{<:PSI.AbstractPTDFModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(
        container,
        PSI.FlowActivePowerVariable,
        network_model,
        devices,
        PSI.StaticBranchUnbounded(),
    )
    PSI.add_parameters!(container, StateEstimationFlows, devices, model)
    PSI.add_feedforward_arguments!(container, model, devices)
    return
end

function PSI.add_parameters!(
    container::PSI.OptimizationContainer,
    ::Type{StateEstimationFlows},
    devices::U,
    model::PSI.DeviceModel{PSY.MonitoredLine, StaticBranchUnboundedStateEstimation},
) where {U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}}} where {D <: PSY.Component}
    time_steps = PSI.get_time_steps(container)
    branch_names = [PSY.get_name(d) for d in devices]
    parameter_container = PSI.add_param_container!(
        container,
        StateEstimationFlows(),
        D,
        ISOPT.VariableKey{PSI.FlowActivePowerVariable, D}(""),
        branch_names,
        time_steps,
    )
    jump_model = PSI.get_jump_model(container)
    for b_name in branch_names, t in time_steps
        PSI.set_multiplier!(parameter_container, 1.0, b_name, t)
        PSI.set_parameter!(parameter_container, jump_model, 0.0, b_name, t)
    end
    return
end

function PSI._make_flow_expressions!(
    jump_model::JuMP.Model,
    name::String,
    time_steps::UnitRange{Int},
    ptdf_col::AbstractVector{Float64},
    nodal_balance_expressions::Matrix{JuMP.AffExpr},
    state_estimation_injections,
    state_estimation_flows,
    sys::PSY.System,
)
    # NOTE - can alternatively avoid passing the system by ensuring the parameter container for state estimation injections
    # is ordered by bus number as is the case for the Active Power Balance expressions
    all_buses = PSY.get_components(
        x -> PSY.get_bustype(x) != PSY.ACBusTypes.ISOLATED,
        PSY.ACBus,
        sys;
    )
    all_bus_numbers = [PSY.get_number(x) for x in all_buses]
    @debug Threads.threadid() name
    state_estimation_flows_parameter = PSI.get_parameter_array(state_estimation_flows)
    state_estimation_flows_multiplier = PSI.get_multiplier_array(state_estimation_flows)
    expressions = Vector{JuMP.AffExpr}(undef, length(time_steps))
    for t in time_steps
        expression = JuMP.@expression(
            jump_model,
            sum(
                ptdf_col[i] * (nodal_balance_expressions[i, t]) for i in 1:length(ptdf_col)
            )
        )
        PSI._add_to_jump_expression!(
            expression,
            state_estimation_flows_parameter[name, t],
            state_estimation_flows_multiplier[name, t],
        )
        JuMP.add_to_expression!(
            expression,
            -1.0 * sum(
                ptdf_col[i] * (state_estimation_injections[string(bus_no), t]) for
                (i, bus_no) in enumerate(sort(all_bus_numbers))
            ),
        )
        expressions[t] = expression
    end
    return name, expressions
    # change when using the not concurrent version
    #return expressions
end

"""
Update parameter function an OperationModel
"""
function PSI.update_container_parameter_values!(
    optimization_container::PSI.OptimizationContainer,
    model::PSI.DecisionModel{MultiRegionProblem},
    key::PSI.ParameterKey{StateEstimationFlows, PSY.MonitoredLine},
    simulation_state::PSI.SimulationState,
)
    # Enable again for detailed debugging
    # TimerOutputs.@timeit RUN_SIMULATION_TIMER "$T $U Parameter Update" begin
    # Note: Do not instantite a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    parameter_array = PSI.get_parameter_array(optimization_container, key)
    parameter_attributes = PSI.get_parameter_attributes(optimization_container, key)
    _update_parameter_values!(
        parameter_array,
        parameter_attributes,
        model,
        simulation_state,
    )
    return
end

function PSI._make_flow_expressions!(
    container::PSI.OptimizationContainer,
    branches::Vector{String},
    time_steps::UnitRange{Int},
    ptdf::PSI.ValidPTDFS,
    nodal_balance_expressions::PSI.JuMPAffineExpressionDArray,
    state_estimation_injections,
    state_estimation_flows,
    branch_Type::DataType,
    sys::PSY.System,
)
    branch_flow_expr = PSI.add_expression_container!(
        container,
        PSI.PTDFBranchFlow(),
        branch_Type,
        branches,
        time_steps,
    )

    jump_model = PSI.get_jump_model(container)

    tasks = map(branches) do name
        ptdf_col = ptdf[name, :]
        Threads.@spawn PSI._make_flow_expressions!(
            jump_model,
            name,
            time_steps,
            ptdf_col,
            nodal_balance_expressions.data,
            state_estimation_injections,
            state_estimation_flows,
            sys,
        )
    end
    for task in tasks
        name, expressions = fetch(task)
        branch_flow_expr[name, :] .= expressions
    end
    # Use serial version for development: 
    #=     for name in branches
            ptdf_col = ptdf[name, :]
            branch_flow_expr[name, :] .= PSI._make_flow_expressions!(
                jump_model,
                name,
                time_steps,
                ptdf_col,
                nodal_balance_expressions.data,
                state_estimation_injections,
                state_estimation_flows,
                sys,
            )
        end =#
    return branch_flow_expr
end

"""
Add network flow constraints for ACBranch and NetworkModel with <: AbstractPTDFModel
"""
function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{NetworkFlowConstraintStateEstimation},
    devices::IS.FlattenIteratorWrapper{B},
    model::PSI.DeviceModel{B, <:PSI.AbstractBranchFormulation},
    network_model::PSI.NetworkModel{<:PSI.AbstractPTDFModel},
    sys::PSY.System,
) where {B <: PSY.ACBranch}
    ptdf = PSI.get_PTDF_matrix(network_model)
    # This is a workaround to not call the same list comprehension to find
    # The subset of branches of type B in the PTDF
    flow_variables = PSI.get_variable(container, PSI.FlowActivePowerVariable(), B)
    branches = flow_variables.axes[1]
    time_steps = PSI.get_time_steps(container)
    branch_flow = PSI.add_constraints_container!(
        container,
        NetworkFlowConstraintStateEstimation(),
        B,
        branches,
        time_steps,
    )
    nodal_balance_expressions =
        PSI.get_expression(container, PSI.ActivePowerBalance(), PSY.ACBus)
    state_estimation_injections =
        PSI.get_parameter_array(container, StateEstimationInjections(), PSY.ACBus)

    state_estimation_flows = PSI.get_parameter(container, StateEstimationFlows(), B)
    flow_variables = PSI.get_variable(container, PSI.FlowActivePowerVariable(), B)
    branch_flow_expr = PSI._make_flow_expressions!(
        container,
        branches,
        time_steps,
        ptdf,
        nodal_balance_expressions,
        state_estimation_injections,
        state_estimation_flows,
        B,
        sys,
    )
    jump_model = PSI.get_jump_model(container)
    for name in branches
        for t in time_steps
            branch_flow[name, t] = JuMP.@constraint(
                jump_model,
                branch_flow_expr[name, t] - flow_variables[name, t] == 0.0
            )
        end
    end
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{PSY.MonitoredLine, StaticBranchUnboundedStateEstimation},
    network_model::PSI.NetworkModel{<:PSI.AbstractPTDFModel},
)
    devices = PSI.get_available_components(model, sys)
    # NOTE - changes required in handling of feedforwards? 
    PSI.add_feedforward_constraints!(container, model, devices)
    PSI.add_constraints!(
        container,
        NetworkFlowConstraintStateEstimation,
        devices,
        model,
        network_model,
        sys,
    )
    PSI.add_constraint_dual!(container, sys, model)
    return
end
