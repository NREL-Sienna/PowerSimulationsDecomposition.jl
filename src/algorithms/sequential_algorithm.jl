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
) end

# The drawback of this approach is that it will loop over the results twice
# once to write into the main container and a second time when writing into the
# store. The upside of this approach is that doesn't require overloading write_model_XXX_results!
# methods from PowerSimulations.
function write_results_to_main_container(container::MultiOptimizationContainer)
    # TODO: This process needs to work in parallel almost right away
    # TODO: This doesn't handle the case where subproblems have an overlap in axis names.

    for subproblem in values(container.subproblems)
        for field in CONTAINER_FIELDS
            subproblem_data_field = getproperty(subproblem, field)
            main_container_data_field = getproperty(container, field)
            for (key, src) in subproblem_data_field
                if src isa JuMP.Containers.SparseAxisArray
                    @warn "Skip SparseAxisArray" field key
                    continue
                end
                num_dims = ndims(src)
                num_dims > 2 && error("ndims = $(num_dims) is not supported yet")
                data = nothing
                try
                    data = PSI.jump_value.(src)
                catch e
                    if e isa UndefRefError
                        @warn "Skip UndefRefError for" field key
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
    end
    # Parameters need a separate approach due to the way the containers work
    return
end

function solve_impl!(
    container::MultiOptimizationContainer{SequentialAlgorithm},
    sys::PSY.System,
)
    # Solve main problem
    status = PSI.RunStatus.SUCCESSFUL
    for (index, subproblem) in container.subproblems
        @info "Solving problem $index"
        status = PSI.solve_impl!(subproblem, sys)
    end
    write_results_to_main_container(container)
    return status
end
