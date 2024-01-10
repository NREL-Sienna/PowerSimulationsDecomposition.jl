function build_impl!(
    container::MultiOptimizationContainer{SequentialAlgorithm},
    template::PSI.ProblemTemplate,
    sys::PSY.System,
)

    for (index, sub_problem) in container.subproblems
        @debug "Building Subproblem $index" _group = PSI.LOG_GROUP_OPTIMIZATION_CONTAINER
        PSI.build_impl!(sub_problem, template, sys)
    end

    build_main_problem!(container, template, sys)

    check_optimization_container(container)

    return
end

function build_main_problem!(
    container::MultiOptimizationContainer{SequentialAlgorithm},
    template::PSI.ProblemTemplate,
    sys::PSY.System)
end

function solve_impl!(container::MultiOptimizationContainer{SequentialAlgorithm}, sys::PSY.System)
    # Solve main problem
    for (index, sub_problem) in container.subproblems
        status = PSI.solve_impl!(sub_problem, sys)
    end
    #write_results_to_main_container()
end
