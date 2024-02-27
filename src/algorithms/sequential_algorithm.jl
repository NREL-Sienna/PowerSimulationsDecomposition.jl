function build_impl!(
    container::MultiOptimizationContainer{SequentialAlgorithm},
    template::MultiProblemTemplate,
    sys::PSY.System,
)
    for (index, sub_problem) in get_sub_templates(template)
        @debug "Building Subproblem $index" _group = PSI.LOG_GROUP_OPTIMIZATION_CONTAINER
        PSI.build_impl!(sub_problem, sub_templates[index], sys)
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

function write_results_to_main_container(container::MultiOptimizationContainer) end

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
    #write_results_to_main_container()
    return status
end
