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

function write_results_to_main_container(container::MultiOptimizationContainer)
    # TODO: This process needs to work in parallel almost right away
    for (index, sub_problem) in container.subproblems
        for field in CONTAINER_FIELDS
            sub_problem_data_field = getproperty(sub_problem, field)
            main_container_data_field = getproperty(container, field)
            for (key, value_container) in sub_problem_data_field
                # write PSI._jump_value() from the value container to the main_container_data_field
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
    for (index, sub_problem) in container.subproblems
        @info "Solving problem $index"
        status = PSI.solve_impl!(sub_problem, sys)
    end
    write_results_to_main_container(container)
    return status
end
