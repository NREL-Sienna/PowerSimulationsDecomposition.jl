abstract type DecompositionAlgorithm end

struct SequentialAlgorithm <: DecompositionAlgorithm end
struct MPIParallelAlgorithm <: DecompositionAlgorithm end

# Formulations

struct SplitAreaPTDFPowerModel <: PSI.AbstractPTDFModel end

# Taken from https://github.com/ANL-CEEESA/UnitCommitment.jl/blob/dev/src/solution/methods/ProgressiveHedging/structs.jl
struct MpiInfo
    comm::Any
    rank::Int
    root::Bool
    nprocs::Int

    function MpiInfo(comm)
        rank = MPI.Comm_rank(comm) + 1
        is_root = (rank == 1)
        nprocs = MPI.Comm_size(comm)
        return new(comm, rank, is_root, nprocs)
    end
end
