"""
Default solve method for MultiOptimizationContainer
"""
function solve_impl!(
    container::MultiOptimizationContainer{MPIParallelAlgorithm},
    system::PSY.System,
)
    # 1. Initialize MPI
    MPI.Init()
    try
        mpi = MpiInfo(MPI.COMM_WORLD)
        # TODO: Look for for loop MPI over dicts to solve subproblems
        solution = solve_subproblem(sp, params, method.inner_method)
        MPI.Barrier(mpi.comm)
        compute_main_problem!(container, mpi, system, solution)
        # Finish loop?
    finally
        update_results!(container, system)
        MPI.Finalize()
    end
    return
end
