struct MultiProblemTemplate <: PSI.AbstractProblemTemplate
    base_template::PSI.ProblemTemplate
    sub_templates::Dict{String, PSI.ProblemTemplate}
end

PSI.get_device_models(template::MultiProblemTemplate) = template.base_template.devices
PSI.get_branch_models(template::MultiProblemTemplate) = template.base_template.branches
PSI.get_service_models(template::MultiProblemTemplate) = template.base_template.services
PSI.get_network_model(template::MultiProblemTemplate) = template.base_template.network_model

function MultiProblemTemplate(
    base_template::PSI.ProblemTemplate,
    problem_keys::Vector{String},
)
    sub_templates = Dict{String, PSI.ProblemTemplate}(
        k => deepcopy(base_template) for k in problem_keys
    )
    return MultiProblemTemplate(base_template, sub_templates)
end

function MultiProblemTemplate(
    network::PSI.NetworkModel{T},
    problem_keys::Vector{String},
) where {T <: PM.AbstractPowerModel}
    return MultiProblemTemplate(PSI.ProblemTemplate(network), problem_keys)
end

function Base.isempty(template::MultiProblemTemplate)
    for template in values(template.sub_templates)
        if !isempty(template.sub_templates)
            return false
        end
    end
    return isempty(template.base_template)
end

function PSI.get_network_formulation(template::MultiProblemTemplate)
    bt = template.base_template
    return PSI.get_network_formulation(PSI.get_network_model(bt))
end

function get_sub_templates(template::MultiProblemTemplate)
    return template.sub_templates
end

function get_sub_problem_keys(template::MultiProblemTemplate)
    return sort!(collect(keys(get_sub_templates(template))))
end

"""
Sets the network model in a template.
"""
function PSI.set_network_model!(
    template::MultiProblemTemplate,
    model::PSI.NetworkModel{<:PM.AbstractPowerModel},
)
    PSI.set_network_model!(template.base_template, model)
    for (id, sub_template) in get_sub_templates(template)
        PSI.set_subsystem!(model, id)
        PSI.set_network_model!(sub_template, model)
    end
    return
end

"""
Sets the device model in a template using the component type and formulation.
Builds a default DeviceModel
"""
function PSI.set_device_model!(
    template::MultiProblemTemplate,
    component_type::Type{<:PSY.Device},
    formulation::Type{<:PSI.AbstractDeviceFormulation},
)
    PSI.set_device_model!(
        template.base_template,
        PSI.DeviceModel(component_type, formulation),
    )
    for (id, sub_template) in get_sub_templates(template)
        network_model = PSI.DeviceModel(component_type, formulation)
        PSI.set_subsystem!(network_model, id)
        PSI.set_device_model!(sub_template, network_model)
    end
    return
end

"""
Sets the device model in a template using a DeviceModel instance
"""
function PSI.set_device_model!(
    template::MultiProblemTemplate,
    model::PSI.DeviceModel{<:PSY.Device, <:PSI.AbstractDeviceFormulation},
)
    PSI.set_device_model!(template.base_template, model)
    for (id, sub_template) in get_sub_templates(template)
        PSI.set_subsystem!(model, id)
        PSI.set_device_model!(sub_template, model)
    end
    return
end

function PSI.set_device_model!(
    template::MultiProblemTemplate,
    model::PSI.DeviceModel{<:PSY.Branch, <:PSI.AbstractDeviceFormulation},
)
    PSI.set_device_model!(template.base_template, model)
    for (id, sub_template) in get_sub_templates(template)
        PSI.set_subsystem!(model, id)
        PSI.set_device_model!(sub_template, PSI.DeviceModel(component_type, formulation))
    end
    return
end

"""
Sets the service model in a template using a name and the service type and formulation.
Builds a default ServiceModel with use_service_name set to true.
"""
function PSI.set_service_model!(
    template::MultiProblemTemplate,
    service_name::String,
    service_type::Type{<:PSY.Service},
    formulation::Type{<:PSI.AbstractServiceFormulation},
)
    PSI.set_service_model!(
        template.base_template,
        service_name,
        ServiceModel(service_type, formulation; use_service_name = true),
    )
    for (id, sub_template) in get_sub_templates(template)
        service_model = ServiceModel(service_type, formulation; use_service_name = true)
        PSI.set_subsystem!(service_model, id)
        PSI.set_service_model!(sub_template, service_name, service_model)
    end
    return
end

"""
Sets the service model in a template using a ServiceModel instance.
"""
function PSI.set_service_model!(
    template::MultiProblemTemplate,
    service_type::Type{<:PSY.Service},
    formulation::Type{<:PSI.AbstractServiceFormulation},
)
    PSI.set_service_model!(
        template.base_template,
        PSI.ServiceModel(service_type, formulation),
    )
    for (id, sub_template) in get_sub_templates(template)
        service_model = ServiceModel(service_type, formulation)
        PSI.set_subsystem!(service_model, id)
        PSI.set_service_model!(sub_template, service_name, service_model)
    end
    return
end

function PSI.set_service_model!(
    template::MultiProblemTemplate,
    service_name::String,
    model::PSI.ServiceModel{<:PSY.Service, <:PSI.AbstractServiceFormulation},
)
    PSI.set_service_model!(template.base_template, service_name, model)
    for (id, sub_template) in get_sub_templates(template)
        PSI.set_subsystem!(model, id)
        PSI.set_service_model!(sub_template, service_name, deepcopy(model))
    end
    return
end

function PSI.set_service_model!(
    template::MultiProblemTemplate,
    model::PSI.ServiceModel{<:PSY.Service, <:PSI.AbstractServiceFormulation},
)
    PSI.set_service_model!(template.base_template, model)
    for (id, sub_template) in get_sub_templates(template)
        PSI.set_subsystem!(model, id)
        PSI.set_service_model!(sub_template, deepcopy(model))
    end
    return
end

function finalize_template!(template::MultiProblemTemplate, sys::PSY.System)
    PSI.finalize_template!(template.base_template, sys)
    for (ix, sub_template) in get_sub_templates(template)
        @debug "Finalizing template for sub probem $ix"
        PSI.finalize_template!(sub_template, sys)
    end
    return
end
