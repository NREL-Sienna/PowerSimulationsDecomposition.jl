function Base.show(io::IO, x::MIME"text/plain", input::MultiProblemTemplate)
    for (k, template) in input.sub_templates
        println(io, "SUBTEMPLATE $k:")
        Base.show(io::IO, x, template)
        println(io, "\n")
    end
end

function Base.show(
    io::IO,
    x::MIME"text/plain",
    input::PSI.DecisionModel{MultiRegionProblem},
)
    Base.show(io::IO, x, input.template)
end
