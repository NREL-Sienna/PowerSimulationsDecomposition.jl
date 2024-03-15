function Base.show(io::IO, ::MIME"text/plain", input::MultiProblemTemplate)
    println(io, "Print somenthing clever here. Template")
end

function Base.show(io::IO, ::MIME"text/plain", input::PSI.DecisionModel{MultiRegionProblem})
    println(io, "Print somenthing clever here. Problem")
end
