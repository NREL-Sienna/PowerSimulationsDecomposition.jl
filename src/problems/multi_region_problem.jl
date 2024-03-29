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

function _join_axes!(axes_data::SortedDict{Int, Set}, ix::Int, axes_value::UnitRange{Int})
    _axes_data = get!(axes_data, ix, Set{UnitRange{Int}}())
    if _axes_data == axes_value
        return
    end
    union!(_axes_data, [axes_value])
    return
end

function _join_axes!(axes_data::SortedDict{Int, Set}, ix::Int, axes_value::Vector)
    _axes_data = get!(axes_data, ix, Set{eltype(axes_value)}())
    union!(_axes_data, axes_value)
    return
end

function _get_axes!(
    common_axes::Dict{Symbol, Dict{PSI.OptimizationContainerKey, SortedDict{Int, Set}}},
    container::PSI.OptimizationContainer,
)
    for field in CONTAINER_FIELDS
        field_data = getfield(container, field)
        for (key, value_container) in field_data
            if isa(value_container, JuMP.Containers.SparseAxisArray)
                continue
            end
            axes_data = get!(common_axes[field], key, SortedDict{Int, Set}())
            for (ix, vals) in enumerate(axes(value_container))
                _join_axes!(axes_data, ix, vals)
            end
        end
    end
    return
end

function _make_joint_axes!(
    dim1::Set{T},
    dim2::Set{UnitRange{Int}},
) where {T <: Union{Int, String}}
    return (collect(dim1), first(dim2))
end

function _make_joint_axes!(dim1::Set{UnitRange{Int}})
    return (first(dim1),)
end

function _map_containers(model::PSI.DecisionModel{MultiRegionProblem})
    common_axes = Dict{Symbol, Dict{PSI.OptimizationContainerKey, SortedDict{Int, Set}}}(
        key => Dict{PSI.OptimizationContainerKey, SortedDict{Int, Set}}() for
        key in CONTAINER_FIELDS
    )
    container = PSI.get_optimization_container(model)
    for subproblem_container in values(container.subproblems)
        _get_axes!(common_axes, subproblem_container)
    end

    for (field, vals) in common_axes
        field_data = getproperty(container, field)
        for (key, axes_data) in vals
            ax = _make_joint_axes!(collect(values(axes_data))...)
            field_data[key] =
                PSI.remove_undef!(JuMP.Containers.DenseAxisArray{Float64}(undef, ax...))
        end
    end
    #TODO: Parameters Requires a different approach

    return
end

function PSI.build_impl!(model::PSI.DecisionModel{MultiRegionProblem})
    build_pre_step!(model)
    @info "Instantiating Network Model"
    instantiate_network_model(model)
    handle_initial_conditions!(model)
    PSI.build_model!(model)
    _map_containers(model)
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

function PSI.serialize_problem(
    model::PSI.DecisionModel{MultiRegionProblem};
    optimizer::Nothing,
) end

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

function PSI._check_numerical_bounds(model::PSI.DecisionModel{MultiRegionProblem}) end
