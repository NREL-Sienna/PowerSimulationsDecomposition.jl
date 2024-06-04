struct MultiRegionProblem <: PSI.DecisionProblem end

function PSI.DecisionModel{MultiRegionProblem}(
    template::MultiProblemTemplate,
    sys::PSY.System,
    ::Union{Nothing, JuMP.Model}=nothing;
    kwargs...,
)
    name = Symbol(get(kwargs, :name, nameof(MultiRegionProblem)))
    settings = PSI.Settings(sys; [k for k in kwargs if first(k) ∉ [:name]]...)
    internal = ISOPT.ModelInternal(
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

    model = PSI.DecisionModel{MultiRegionProblem}(
        name,
        template_,
        sys,
        internal,
        PSI.SimulationInfo(),
        PSI.DecisionModelStore(),
        Dict{String, Any}(),
    )
    PSI.validate_time_series!(model)
    return model
end

function _join_axes!(axes_data::OrderedDict{Int, Set}, ix::Int, axes_value::UnitRange{Int})
    _axes_data = get!(axes_data, ix, Set{UnitRange{Int}}())
    if _axes_data == axes_value
        return
    end
    union!(_axes_data, [axes_value])
    return
end

function _join_axes!(axes_data::OrderedDict{Int, Set}, ix::Int, axes_value::Vector)
    _axes_data = get!(axes_data, ix, Set{eltype(axes_value)}())
    union!(_axes_data, axes_value)
    return
end

function _get_axes!(
    common_axes::Dict{Symbol, Dict{PSI.OptimizationContainerKey, OrderedDict{Int, Set}}},
    container::PSI.OptimizationContainer,
)
    for field in CONTAINER_FIELDS
        field_data = getfield(container, field)
        for (key, value_container) in field_data
            if isa(value_container, JuMP.Containers.SparseAxisArray)
                continue
            end
            axes_data = get!(common_axes[field], key, OrderedDict{Int, Set}())
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
    common_axes = Dict{Symbol, Dict{PSI.OptimizationContainerKey, OrderedDict{Int, Set}}}(
        key => Dict{PSI.OptimizationContainerKey, OrderedDict{Int, Set}}() for
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
            field_data[key] = JuMP.Containers.DenseAxisArray{Float64}(undef, ax...)
        end
    end

    _make_parameter_container!(model)
    return
end

function _make_parameter_container!(model)
    container = PSI.get_optimization_container(model)
    subproblem_parameters = [x.parameters for x in values(container.subproblems)]
    parameter_arrays = _make_parameter_arrays(subproblem_parameters, :parameter_array)
    multiplier_arrays = _make_parameter_arrays(subproblem_parameters, :multiplier_array)
    attributes = _make_parameter_attributes(subproblem_parameters)

    !issetequal(keys(parameter_arrays), keys(multiplier_arrays)) &&
        error("Bug: key mismatch")
    !issetequal(keys(parameter_arrays), keys(attributes)) && error("Bug: key mismatch")

    for key in keys(parameter_arrays)
        container.parameters[key] = PSI.ParameterContainer(
            attributes[key],
            parameter_arrays[key],
            multiplier_arrays[key],
        )
    end
end

function _make_parameter_attributes(subproblem_parameters)
    data = Dict()
    for parameters in subproblem_parameters
        for (key, val) in parameters
            if !haskey(data, key)
                data[key] = deepcopy(val.attributes)
            else
                _merge_attributes!(data[key], val.attributes)
            end
        end
    end

    return data
end

function _merge_attributes!(attributes::T, other::T) where {T <: PSI.ParameterAttributes}
    for field in fieldnames(T)
        val1 = getproperty(attributes, field)
        val2 = getproperty(other, field)
        if val1 != val2
            error(
                "Mismatch in attributes values. T = $T attributes = $attributes other = $other",
            )
        end
    end
end

function _merge_attributes!(attributes::T, other::T) where {T <: PSI.TimeSeriesAttributes}
    if attributes.name != other.name
        error("Mismatch in attributes name: $(attributes.name) $(other.name)")
    end
    intersection = intersect(
        keys(attributes.component_name_to_ts_uuid),
        keys(other.component_name_to_ts_uuid),
    )
    if !isempty(intersection)
        error("attributes component_name_to_ts_uuid have collsions: $intersection")
    end

    merge!(attributes.component_name_to_ts_uuid, other.component_name_to_ts_uuid)
    return
end

function _make_parameter_arrays(subproblem_parameters, field_name)
    data = Dict{PSI.ParameterKey, AbstractArray}()
    for parameters in subproblem_parameters
        for (key, val) in parameters
            if val isa JuMP.Containers.SparseAxisArray
                @warn "Skipping SparseAxisArray"  # TODO
                continue
            end
            array = getproperty(val, field_name)
            if !haskey(data, key)
                data[key] = JuMP.Containers.DenseAxisArray{Float64}(undef, axes(array)...)
            else
                existing = data[key]
                data[key] = _make_array_joined_by_axes(existing, array)
            end
        end
    end

    return data
end

function _make_array_joined_by_axes(
    a1::JuMP.Containers.DenseAxisArray{T, 2},
    a2::JuMP.Containers.DenseAxisArray{U, 2},
) where {T <: Union{Float64, JuMP.VariableRef}, U <: Union{Float64, JuMP.VariableRef}}
    ax1 = axes(a1)
    ax2 = axes(a2)
    if ax1[2] != ax2[2]
        error("axis 2 must be the same")
    end

    if issetequal(ax1[1], ax2[1])
        return JuMP.Containers.DenseAxisArray{Float64}(undef, ax1[1], ax1[2])
    end

    axis1 = union(ax1[1], ax2[1])
    return JuMP.Containers.DenseAxisArray{Float64}(undef, axis1, ax1[2])
end

function PSI.build_impl!(model::PSI.DecisionModel{MultiRegionProblem})
    build_pre_step!(model)
    @info "Instantiating Network Model"
    instantiate_network_model(model)
    handle_initial_conditions!(model)
    PSI.build_model!(model)
    _map_containers(model)
    container = PSI.get_optimization_container(model)
    container.built_for_recurrent_solves = true
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
    PSI.set_status!(model, ISOPT.ModelBuildStatus.IN_PROGRESS)
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

### Simulation Related methods ###
# These code blocks are duplicative from PSI, refactoring might be required on the PSI side to
# avoid duplication.

function PSI._add_feedforward_to_model(
    sim_model::PSI.DecisionModel{MultiRegionProblem},
    ff::T,
    ::Type{U},
) where {T <: PSI.AbstractAffectFeedforward, U <: PSY.Device}
    template = PSI.get_template(sim_model)
    for (id, sub_template) in get_sub_templates(template)
        device_model = PSI.get_model(sub_template, PSI.get_component_type(ff))
        if device_model === nothing
            model_name = PSI.get_name(sim_model)
            throw(
                IS.ConflictingInputsError(
                    "Device model $(PSI.get_component_type(ff)) not found in model $model_name",
                ),
            )
        end
        @info "Attaching $T to $(PSI.get_component_type(ff)) to Template $id"
        PSI.attach_feedforward!(device_model, ff)
    end
    return
end

function PSI._add_feedforward_to_model(
    sim_model::PSI.DecisionModel{MultiRegionProblem},
    ff::T,
    ::Type{U},
) where {T <: PSI.AbstractAffectFeedforward, U <: PSY.Service}
    template = PSI.get_template(sim_model)
    name_provided = PSI.get_feedforward_meta(ff) != PSI.NO_SERVICE_NAME_PROVIDED
    for (id, sub_template) in get_sub_templates(template)
        if name_provided
            service_model = PSI.get_model(
                sub_template,
                PSI.get_component_type(ff),
                PSI.get_feedforward_meta(ff),
            )
            if service_model === nothing
                throw(
                    IS.ConflictingInputsError(
                        "Service model $(get_component_type(ff)) not found in model $(get_name(sim_model))",
                    ),
                )
            end
            @info "Attaching $T to $(PSI.get_component_type(ff)) $(PSI.get_feedforward_meta(ff)) to Template $id"
            PSI.attach_feedforward!(service_model, ff)
        else
            service_found = false
            for (key, model) in PSI.get_service_models(sub_template)
                if key[2] == Symbol(PSI.get_component_type(ff))
                    service_found = true
                    @info "Attaching $T to $(PSI.get_component_type(ff))"
                    PSI.attach_feedforward!(model, ff)
                end
            end
        end
    end
    return
end

function PSI.update_parameters!(
    model::PSI.DecisionModel{MultiRegionProblem},
    decision_states::PSI.DatasetContainer{PSI.InMemoryDataset},
)
    container = PSI.get_optimization_container(model)
    for (ix, subproblem) in container.subproblems
        @debug "Updating subproblem $ix"
        PSI.cost_function_unsynch(subproblem)
        for key in keys(PSI.get_parameters(subproblem))
            PSI.update_container_parameter_values!(subproblem, model, key, decision_states)
        end
    end

    if !PSI.is_synchronized(model)
        for subproblem in values(container.subproblems)
            PSI.update_objective_function!(subproblem)
            obj_func = PSI.get_objective_expression(subproblem)
            PSI.set_synchronized_status(obj_func, true)
        end
    end
    return
end

"""
Default problem update function for most problems with no customization
"""
function PSI.update_model!(
    model::PSI.DecisionModel{MultiRegionProblem},
    sim::PSI.Simulation,
)
    PSI.update_model!(
        model,
        PSI.get_simulation_state(sim),
        PSI.get_ini_cond_chronology(sim),
    )
    #=
    if get_rebuild_model(model)
        container = get_optimization_container(model)
        reset_optimization_model!(container)
        build_impl!(container, get_template(model), get_system(model))
    end
    =#
    return
end
