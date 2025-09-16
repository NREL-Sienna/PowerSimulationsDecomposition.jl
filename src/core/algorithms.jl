abstract type DecompositionAlgorithm end

struct SequentialAlgorithm <: DecompositionAlgorithm end
struct MPIParallelAlgorithm <: DecompositionAlgorithm end
