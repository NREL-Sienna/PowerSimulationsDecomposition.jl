abstract type CoordinationAlgorithm end

struct NoCoordination <: CoordinationAlgorithm end

"""
Initialize coordination for a subproblem during the build phase.
"""
function initialize_coordination!(
    ::CoordinationAlgorithm,
    ::PSI.OptimizationContainer,
    ::PSY.System,
)
    return
end