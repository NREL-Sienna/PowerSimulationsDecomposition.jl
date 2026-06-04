function initialize_coordination!(
    ::NoCoordination,
    ::PSI.OptimizationContainer,
    ::PSY.System,
)
    return
end

function update_coordination!(
    ::NoCoordination,
    ::MultiOptimizationContainer,
    ::PSY.System,
    ::String,
)
    return
end

function apply_coordination!(
    ::NoCoordination,
    ::MultiOptimizationContainer,
    ::PSY.System,
)
    return
end