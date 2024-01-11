function system_modification(sys::PSY.System, index)
    for component in get_components(Component, sys)
        ext = get_ext(component)
        if !haskey(ext, "subsystems")
            continue
        else
            # needs to be careful with he buses becasue buses don't have "available"
            if index in ext["subsystems"]
                set_available!(component, false)
            else
                set_available!(component, true)
            end
        end
    return
end

function build_impl!(
    container::MultiOptimizationContainer{SequentialAlgorithm},
    template::PSI.ProblemTemplate,
    sys::PSY.System,
)

    for (index, sub_problem) in container.subproblems
        @debug "Building Subproblem $index" _group = PSI.LOG_GROUP_OPTIMIZATION_CONTAINER
        # Temporary
        system_modification!(sys, index)
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

function write_results_to_main_container(container::MultiOptimizationContainer)

end

function solve_impl!(container::MultiOptimizationContainer{SequentialAlgorithm}, sys::PSY.System)
    # Solve main problem
    status = PSI.RunStatus.SUCCESSFUL
    for (index, sub_problem) in container.subproblems
        @info "Solving problem $index"
        status = PSI.solve_impl!(sub_problem, sys)
    end
    #write_results_to_main_container()
    return status
end
