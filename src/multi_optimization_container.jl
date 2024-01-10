mutable struct MultiOptimizationContainer{T<:DecompositionAlgorithm} <: PSI.AbstractModelContainer
    main_JuMPmodel::JuMP.Model
    subproblems::Dict{String, PSI.OptimizationContainer}
    time_steps::UnitRange{Int}
    resolution::Dates.TimePeriod
    settings::PSI.Settings
    settings_copy::PSI.Settings
    variables::Dict{PSI.VariableKey, AbstractArray}
    aux_variables::Dict{PSI.AuxVarKey, AbstractArray}
    duals::Dict{PSI.ConstraintKey, AbstractArray}
    constraints::Dict{PSI.ConstraintKey, AbstractArray}
    objective_function::PSI.ObjectiveFunction
    expressions::Dict{PSI.ExpressionKey, AbstractArray}
    parameters::Dict{PSI.ParameterKey, PSI.ParameterContainer}
    primal_values_cache::PSI.PrimalValuesCache
    initial_conditions::Dict{PSI.ICKey, Vector{<:PSI.InitialCondition}}
    initial_conditions_data::PSI.InitialConditionsData
    infeasibility_conflict::Dict{Symbol, Array}
    pm::Union{Nothing, PM.AbstractPowerModel}
    base_power::Float64
    optimizer_stats::PSI.OptimizerStats
    built_for_recurrent_solves::Bool
    metadata::PSI.OptimizationContainerMetadata # Unclear about how to extend this correctly
    default_time_series_type::Type{<:PSY.TimeSeriesData}  # Maybe isn't needed here
    mpi_info::Union{Nothing, MpiInfo}
end

function MultiOptimizationContainer(
    ::Type{T},
    sys::PSY.System,
    settings::PSI.Settings,
    ::Type{U},
    sub_problem_keys::Vector{String}
) where {T <: DecompositionAlgorithm, U <: PSY.TimeSeriesData}
    resolution = PSY.get_time_series_resolution(sys)
    if isabstracttype(U)
        error("Default Time Series Type $U can't be abstract")
    end

    subproblems = Dict{String, PSI.OptimizationContainer}()
    for k in sub_problem_keys
        subproblems[k] = PSI.OptimizationContainer(sys, settings, nothing, U)
    end

    return MultiOptimizationContainer{T}(
        JuMP.Model(),
        subproblems,
        1:1,
        IS.time_period_conversion(resolution),
        settings,
        PSI.copy_for_serialization(settings),
        Dict{PSI.VariableKey, AbstractArray}(),
        Dict{PSI.AuxVarKey, AbstractArray}(),
        Dict{PSI.ConstraintKey, AbstractArray}(),
        Dict{PSI.ConstraintKey, AbstractArray}(),
        PSI.ObjectiveFunction(),
        Dict{PSI.ExpressionKey, AbstractArray}(),
        Dict{PSI.ParameterKey, PSI.ParameterContainer}(),
        PSI.PrimalValuesCache(),
        Dict{PSI.ICKey, Vector{PSI.InitialCondition}}(),
        PSI.InitialConditionsData(),
        Dict{Symbol, Array}(),
        nothing,
        PSY.get_base_power(sys),
        PSI.OptimizerStats(),
        false,
        PSI.OptimizationContainerMetadata(),
        U,
        nothing,
    )
end

function get_container_keys(container::MultiOptimizationContainer)
    return Iterators.flatten(keys(getfield(container, f)) for f in STORE_CONTAINERS)
end

PSI.get_default_time_series_type(container::MultiOptimizationContainer) =
    container.default_time_series_type
PSI.get_duals(container::MultiOptimizationContainer) = container.duals
PSI.get_expressions(container::MultiOptimizationContainer) = container.expressions
PSI.get_infeasibility_conflict(container::MultiOptimizationContainer) =
    container.infeasibility_conflict
PSI.get_initial_conditions(container::MultiOptimizationContainer) = container.initial_conditions
PSI.get_initial_conditions_data(container::MultiOptimizationContainer) =
    container.initial_conditions_data
PSI.get_initial_time(container::MultiOptimizationContainer) = PSI.get_initial_time(container.settings)
PSI.get_jump_model(container::MultiOptimizationContainer) = container.main_JuMPmodel
PSI.get_metadata(container::MultiOptimizationContainer) = container.metadata
PSI.get_optimizer_stats(container::MultiOptimizationContainer) = container.optimizer_stats
PSI.get_parameters(container::MultiOptimizationContainer) = container.parameters
PSI.get_resolution(container::MultiOptimizationContainer) = container.resolution
PSI.get_settings(container::MultiOptimizationContainer) = container.settings
PSI.get_time_steps(container::MultiOptimizationContainer) = container.time_steps
PSI.get_variables(container::MultiOptimizationContainer) = container.variables

PSI.set_initial_conditions_data!(container::MultiOptimizationContainer, data) =
    container.initial_conditions_data = data
    PSI.get_objective_expression(container::MultiOptimizationContainer) = container.objective_function
    PSI.is_synchronized(container::MultiOptimizationContainer) =
    container.objective_function.synchronized
PSI.set_time_steps!(container::MultiOptimizationContainer, time_steps::UnitRange{Int64}) =
    container.time_steps = time_steps

PSI.get_aux_variables(container::MultiOptimizationContainer) = container.aux_variables
PSI.get_base_power(container::MultiOptimizationContainer) = container.base_power
PSI.get_constraints(container::MultiOptimizationContainer) = container.constraints


function check_optimization_container(container::MultiOptimizationContainer)
    # Solve main problem
    for (index, sub_problem) in container.subproblems
        PSI.check_optimization_container(sub_problem)
    end
end

function _finalize_jump_model!(container::MultiOptimizationContainer, settings::PSI.Settings)
    @debug "Instantiating the JuMP model" _group = PSI.LOG_GROUP_OPTIMIZATION_CONTAINER
    #=
    if PSI.built_for_recurrent_solves(container) && PSI.get_optimizer(settings) === nothing
        throw(
            IS.ConflictingInputsError(
                "Optimizer can not be nothing when building for recurrent solves",
            ),
        )
    end
    =#

    if PSI.get_direct_mode_optimizer(settings)
        optimizer = () -> MOI.instantiate(PSI.get_optimizer(settings))
        container.main_JuMPmodel = JuMP.direct_model(optimizer())
    elseif PSI.get_optimizer(settings) === nothing
        @debug "The optimization model has no optimizer attached" _group =
            LOG_GROUP_OPTIMIZATION_CONTAINER
    else
        JuMP.set_optimizer(PSI.get_jump_model(container), PSI.get_optimizer(settings))
    end

    JuMPmodel = PSI.get_jump_model(container)

    if PSI.get_optimizer_solve_log_print(settings)
        JuMP.unset_silent(JuMPmodel)
        @debug "optimizer unset to silent" _group = PSI.LOG_GROUP_OPTIMIZATION_CONTAINER
    else
        JuMP.set_silent(JuMPmodel)
        @debug "optimizer set to silent" _group = PSI.LOG_GROUP_OPTIMIZATION_CONTAINER
    end
    return
end

function init_optimization_container!(
    container::MultiOptimizationContainer,
    ::Type{T},
    sys::PSY.System,
) where {T <: PM.AbstractPowerModel}
    PSY.set_units_base_system!(sys, "SYSTEM_BASE")
    # The order of operations matter
    settings = PSI.get_settings(container)

    if PSI.get_initial_time(settings) == PSI.UNSET_INI_TIME
        if PSI.get_default_time_series_type(container) <: PSY.AbstractDeterministic
            PSI.set_initial_time!(settings, PSY.get_forecast_initial_timestamp(sys))
        elseif PSI.get_default_time_series_type(container) <: PSY.SingleTimeSeries
            ini_time, _ = PSY.check_time_series_consistency(sys, PSY.SingleTimeSeries)
            PSI.set_initial_time!(settings, ini_time)
        end
    end

    if PSI.get_horizon(settings) == PSI.UNSET_HORIZON
        PSI.set_horizon!(settings, PSY.get_forecast_horizon(sys))
    end
    container.time_steps = 1:PSI.get_horizon(settings)

    stats = PSI.get_optimizer_stats(container)
    stats.detailed_stats = PSI.get_detailed_optimizer_stats(settings)

    _finalize_jump_model!(container, settings)

    for (index, sub_problem) in container.subproblems
        @debug "Initializing Container Subproblem $index" _group = PSI.LOG_GROUP_OPTIMIZATION_CONTAINER
        sub_problem.settings = deepcopy(settings)
        PSI.init_optimization_container!(sub_problem, T, sys)
    end

    return
end
