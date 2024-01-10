module PowerSimulationsDecomposition

import PowerSimulations
import JuMP
import Dates
import MPI

const PSI = PowerSimulations
const PM = PSI.PM
const PSY = PSI.PSY
const IS = PSI.IS

using DocStringExtensions
@template (FUNCTIONS, METHODS) = """
                                 $(TYPEDSIGNATURES)
                                 $(DOCSTRING)
                                 """


include("core.jl")
include("multi_optimization_container.jl")
include("algorithms/sequential_algorithm.jl")
include("algorithms/mpi_parallel_algorithm.jl")
include("problems/multi_region_problem.jl")

end
