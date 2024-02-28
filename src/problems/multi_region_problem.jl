struct MultiRegionProblem <: PSI.DecisionProblem end

function PSI.DecisionModel{MultiRegionProblem}(
    template::MultiProblemTemplate,
    sys::PSY.System,
    ::Union{Nothing, JuMP.Model}=nothing;
    kwargs...,
)
    name = Symbol(get(kwargs, :name, nameof(MultiRegionProblem)))
    settings = PSI.Settings(sys; [k for k in kwargs if first(k) ∉ [:name]]...)
    internal = PSI.ModelInternal(
        MultiOptimizationContainer(
            SequentialAlgorithm,
            sys,
            settings,
            PSY.Deterministic,
            get_sub_problem_keys(template),
        ),
    )
    template_ = deepcopy(template)

    finalize_template!(template_, sys)

    # return multi-region decision model container
    return PSI.DecisionModel{MultiRegionProblem}(
        name,
        template_,
        sys,
        internal,
        PSI.DecisionModelStore(),
        Dict{String, Any}(),
    )
end

function PSI.build_impl!(model::PSI.DecisionModel{MultiRegionProblem})
    build_pre_step!(model)
    @info "Instantiating Network Model"
    instantiate_network_model(model)
    handle_initial_conditions!(model)
    PSI.build_model!(model)
    # Might need custom implementation for this container type
    # serialize_metadata!(get_optimization_container(model), get_output_dir(model))
    PSI.log_values(PSI.get_settings(model))
    return
end

function build_pre_step!(model::PSI.DecisionModel{MultiRegionProblem})
    @info "Initializing Optimization Container For a DecisionModel"
    init_optimization_container!(
        PSI.get_optimization_container(model),
        PSI.get_network_model(PSI.get_template(model)),
        PSI.get_system(model),
    )
    @info "Initializing ModelStoreParams"
    PSI.init_model_store_params!(model)
    PSI.set_status!(model, PSI.BuildStatus.IN_PROGRESS)
    return
end

function handle_initial_conditions!(model::PSI.DecisionModel{MultiRegionProblem}) end

function instantiate_network_model(model::PSI.DecisionModel{MultiRegionProblem})
    template = PSI.get_template(model)
    for (id, sub_template) in get_sub_templates(template)
        network_model = PSI.get_network_model(sub_template)
        PSI.set_subsystem!(network_model, id)
        PSI.instantiate_network_model(network_model, PSI.get_system(model))
    end
    return
end

function PSI.build_model!(model::PSI.DecisionModel{MultiRegionProblem})
    build_impl!(
        PSI.get_optimization_container(model),
        PSI.get_template(model),
        PSI.get_system(model),
    )
    return
end

function PSI.solve_impl!(model::PSI.DecisionModel{MultiRegionProblem})
    status = solve_impl!(PSI.get_optimization_container(model), PSI.get_system(model))
    PSI.set_run_status!(model, status)
    return
end

function PSI.write_model_dual_results!(
    store,
    model::PSI.DecisionModel{MultiRegionProblem},
    index::PSI.DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol, Any}, Nothing},
) end
function PSI.write_model_parameter_results!(
    store,
    model::PSI.DecisionModel{MultiRegionProblem},
    index::PSI.DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol, Any}, Nothing},
) end
function PSI.write_model_variable_results!(
    store,
    model::PSI.DecisionModel{MultiRegionProblem},
    index::PSI.DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol, Any}, Nothing},
) end
function PSI.write_model_aux_variable_results!(
    store,
    model::PSI.DecisionModel{MultiRegionProblem},
    index::PSI.DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol, Any}, Nothing},
) end
function PSI.write_model_expression_results!(
    store,
    model::PSI.DecisionModel{MultiRegionProblem},
    index::PSI.DecisionModelIndexType,
    update_timestamp::Dates.DateTime,
    export_params::Union{Dict{Symbol, Any}, Nothing},
) end
