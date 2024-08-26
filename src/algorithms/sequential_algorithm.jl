function build_impl!(
    container::MultiOptimizationContainer{SequentialAlgorithm},
    template::MultiProblemTemplate,
    sys::PSY.System,
)
    for (index, sub_template) in get_sub_templates(template)
        @info "Building Subproblem $index" _group = PSI.LOG_GROUP_OPTIMIZATION_CONTAINER
        PSI.build_impl!(get_subproblem(container, index), sub_template, sys)
    end

    build_main_problem!(container, template, sys)

    check_optimization_container(container)

    return
end

function build_main_problem!(
    container::MultiOptimizationContainer{SequentialAlgorithm},
    template::MultiProblemTemplate,
    sys::PSY.System,
)
end

# The drawback of this approach is that it will loop over the results twice
# once to write into the main container and a second time when writing into the
# store. The upside of this approach is that doesn't require overloading write_model_XXX_results!
# methods from PowerSimulations.
function write_results_to_main_container(container::MultiOptimizationContainer)
    # TODO: This process needs to work in parallel almost right away
    # TODO: This doesn't handle the case where subproblems have an overlap in axis names.

    for (k, subproblem) in container.subproblems
        for field in CONTAINER_FIELDS
            subproblem_data_field = getproperty(subproblem, field)
            main_container_data_field = getproperty(container, field)
            for (key, src) in subproblem_data_field
                if src isa JuMP.Containers.SparseAxisArray
                    @debug "Skip SparseAxisArray" field key
                    continue
                end
                num_dims = ndims(src)
                num_dims > 2 && error("ndims = $(num_dims) is not supported yet")
                data = nothing
                try
                    if key == ISOPT.ExpressionKey{PSI.ActivePowerBalance, PSY.ACBus}("")
                        @error "mt exp $k" data = PSI.jump_value.(src)[10313, :]
                    end
                    data = PSI.jump_value.(src)
                catch e
                    if e isa UndefRefError
                        @error "Skip UndefRefError for" field key
                        continue
                    end
                    rethrow()
                end
                dst = main_container_data_field[key]
                if num_dims == 1
                    dst[1:length(axes(src)[1])] = data
                elseif num_dims == 2
                    columns = axes(src)[1]
                    len = length(axes(src)[2])
                    dst[columns, 1:len] = PSI.jump_value.(src[:, :])
                elseif num_dims == 3
                    # TODO: untested
                    axis1 = axes(src)[1]
                    axis2 = axes(src)[2]
                    len = length(axes(src)[3])
                    dst[axis1, axis2, 1:len] = PSI.jump_value.(src[:, :, :])
                end
            end
        end
        _write_parameter_results_to_main_container(container, subproblem)
    end
    return
end

function _write_parameter_results_to_main_container(
    container::MultiOptimizationContainer,
    subproblem,
)
    for (key, parameter_container) in subproblem.parameters
        num_dims = ndims(parameter_container.parameter_array)
        num_dims > 2 && error("ndims = $(num_dims) is not supported yet")
        src_param_data = PSI.jump_value.(parameter_container.parameter_array)
        src_mult_data = PSI.jump_value.(parameter_container.multiplier_array)
        dst_param_data = container.parameters[key].parameter_array
        dst_mult_data = container.parameters[key].multiplier_array
        if num_dims == 1
            dst_param_data[1:length(axes(src_param_data)[1])] = src_param_data
            dst_mult_data[1:length(axes(src_mult_data)[1])] = src_mult_data
        elseif num_dims == 2
            param_columns = axes(src_param_data)[1]
            mult_columns = axes(src_mult_data)[1]
            len = length(axes(src_param_data)[2])
            @assert_op len == length(axes(src_mult_data)[2])
            dst_param_data[param_columns, 1:len] = PSI.jump_value.(src_param_data[:, :])
            dst_mult_data[mult_columns, 1:len] = PSI.jump_value.(src_mult_data[:, :])
        else
            error("Bug")
        end
    end
end

function solve_impl!(
    container::MultiOptimizationContainer{SequentialAlgorithm},
    sys::PSY.System,
)
    # Solve main problem
    status = ISSIM.RunStatus.RUNNING
    for (index, subproblem) in container.subproblems
        @debug "Solving problem $index"
        status = PSI.solve_impl!(subproblem, sys)
        if status != ISSIM.RunStatus.SUCCESSFULLY_FINALIZED
            return status
        end
    end
    write_results_to_main_container(container)
    return status
end
