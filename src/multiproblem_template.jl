struct MultiProblemTemplate <: PSI.AbstractProblemTemplate
    base_template::PSI.ProblemTemplate
    sub_templates::Dict{String, PSI.ProblemTemplate}
end

function Base.isempty(template::MultiProblemTemplate)
    for template in values(template.sub_templates)
        if !isempty(template.sub_templates)
            return false
        end
    end
    return isempty(template.base_template)
end

function MultiProblemTemplate(
    base_template::PSI.ProblemTemplate,
    problem_keys::Vector{String},
)
    sub_templates = Dict{String, PSI.ProblemTemplate}(k => deepcopy(base_template) for k in problem_keys)
    return MultiProblemTemplate(base_template, sub_templates)
end

function MultiProblemTemplate(
    network::PSI.NetworkModel{T},
    problem_keys::Vector{String},) where {T <: PM.AbstractPowerModel}
    return MultiProblemTemplate(PSI.ProblemTemplate(network), problem_keys)
end

function get_sub_templates(template::MultiProblemTemplate)
    return values(template.sub_templates)
end

"""
Sets the network model in a template.
"""
function PSI.set_network_model!(
    template::MultiProblemTemplate,
    model::PSI.NetworkModel{<:PM.AbstractPowerModel},
)
    PSI.set_network_model!(template.base_template, model)
    for sub_template in get_sub_templates(template)
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
    PSI.set_device_model!(template.base_template, PSI.DeviceModel(component_type, formulation))
    for sub_template in get_sub_templates(template)
        PSI.set_device_model!(sub_template, PSI.DeviceModel(component_type, formulation))
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
    for sub_template in get_sub_templates(template)
        PSI.set_device_model!(sub_template, model)
    end
    return
end

function PSI.set_device_model!(
    template::MultiProblemTemplate,
    model::PSI.DeviceModel{<:PSY.Branch, <:PSI.AbstractDeviceFormulation},
)
    PSI.set_device_model!(template.base_template, model)
    for sub_template in get_sub_templates(template)
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
    for sub_template in get_sub_templates(template)
        PSI.set_service_model!(
            sub_template,
            service_name,
            ServiceModel(service_type, formulation; use_service_name = true),
        )
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
    PSI.set_service_model!(template.base_template, PSI.ServiceModel(service_type, formulation))
    for sub_template in get_sub_templates(template)
        PSI.set_service_model!(
            sub_template,
            service_name,
            PSI.ServiceModel(service_type, formulation),
        )
    end
    return
end

function PSI.set_service_model!(
    template::MultiProblemTemplate,
    service_name::String,
    model::PSI.ServiceModel{<:PSY.Service, <:PSI.AbstractServiceFormulation},
)
    PSI.set_service_model!(template.base_template, service_name, model)
    for sub_template in get_sub_templates(template)
        PSI.set_service_model!(sub_template, service_name, model)
    end
    return
end

function PSI.set_service_model!(
    template::MultiProblemTemplate,
    model::PSI.ServiceModel{<:PSY.Service, <:PSI.AbstractServiceFormulation},
)
    PSI.set_service_model!(template.base_template, model)
    for sub_template in get_sub_templates(template)
        PSI.set_service_model!(sub_template, model)
    end
    return
end

function PSI.finalize_template!(template::MultiProblemTemplate, sys::PSY.System)
    for sub_template in get_sub_templates(template)
        PSI.finalize_template!(sub_template, sys)
    end
end
