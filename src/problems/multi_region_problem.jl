struct MultiRegionProblem <: PSI.DecisionProblem end

function PSI.DecisionModel{MultiRegionProblem}(
    template::PSI.ProblemTemplate,
    sys::PSY.System,
    settings::PSI.Settings,
    ::Union{Nothing,JuMP.Model}=nothing;
    name=nothing,
)
    if name === nothing
        name = nameof(MultiRegionProblem)
    elseif name isa String
        name = Symbol(name)
    end
    # Get these from the system later with subsystems
    region_keys = ["1", "2,"]
    internal = PSI.ModelInternal(
        MultiOptimizationContainer(SequentialAlgorithm, sys, settings, PSY.Deterministic, region_keys),
    )
    template_ = deepcopy(template)
    PSI.finalize_template!(template_, sys)
    return PSI.DecisionModel{MultiRegionProblem}(
        name,
        template_,
        sys,
        internal,
        PSI.DecisionModelStore(),
        Dict{String,Any}(),
    )
end

function PSI.build_impl!(model::PSI.DecisionModel{MultiRegionProblem})
    build_pre_step!(model)
    @info "Instantiating Network Model"
    instantiate_network_model(model)
    handle_initial_conditions!(model)
    PSI.build_model!(model)
    # Might need custom implementation for this container type
    #serialize_metadata!(get_optimization_container(model), get_output_dir(model))
    PSI.log_values(PSI.get_settings(model))
    return
end

function build_pre_step!(model::PSI.DecisionModel{MultiRegionProblem})
    @info "Initializing Optimization Container For a DecisionModel"
    init_optimization_container!(
        PSI.get_optimization_container(model),
        PSI.get_network_formulation(PSI.get_template(model)),
        PSI.get_system(model),
    )
    @info "Initializing ModelStoreParams"
    PSI.init_model_store_params!(model)
    PSI.set_status!(model, PSI.BuildStatus.IN_PROGRESS)
    return
end

function handle_initial_conditions!(model::PSI.DecisionModel{MultiRegionProblem})
end

function instantiate_network_model(model::PSI.DecisionModel{MultiRegionProblem})
    PSI.instantiate_network_model(model)
    return
end

function PSI.build_model!(model::PSI.DecisionModel{MultiRegionProblem})
    build_impl!(PSI.get_optimization_container(model), PSI.get_template(model), PSI.get_system(model))
    return
end

function PSI.solve_impl!(model::PSI.DecisionModel{MultiRegionProblem})
    status = solve_impl!(PSI.get_optimization_container(model), PSI.get_system(model))
    PSI.set_run_status!(model, status)
    return
end

function PSI.write_model_dual_results!(store,
    model::PSI.DecisionModel{MultiRegionProblem},
    index::PSI.DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol,Any},Nothing},)
end
function PSI.write_model_parameter_results!(store,
    model::PSI.DecisionModel{MultiRegionProblem},
    index::PSI.DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol,Any},Nothing},)
end
function PSI.write_model_variable_results!(store,
    model::PSI.DecisionModel{MultiRegionProblem},
    index::PSI.DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol,Any},Nothing},)
end
function PSI.write_model_aux_variable_results!(store,
    model::PSI.DecisionModel{MultiRegionProblem},
    index::PSI.DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol,Any},Nothing},)
end
function PSI.write_model_expression_results!(store,
    model::PSI.DecisionModel{MultiRegionProblem},
    index::PSI.DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol,Any},Nothing},)
end
