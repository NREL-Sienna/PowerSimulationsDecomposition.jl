module PowerSimulationsDecomposition

export MultiRegionProblem
export MultiProblemTemplate

import PowerSimulations
import PowerSystems
import InfrastructureSystems
import JuMP
import Dates
import MPI
import MathOptInterface

const PSI = PowerSimulations
const PSY = PowerSystems
const IS = InfrastructureSystems
const PM = PSI.PM
const MOI = MathOptInterface

using DocStringExtensions
@template (FUNCTIONS, METHODS) = """
                                 $(TYPEDSIGNATURES)
                                 $(DOCSTRING)
                                 """

include("core.jl")
include("multiproblem_template.jl")
include("multi_optimization_container.jl")
include("algorithms/sequential_algorithm.jl")
include("algorithms/mpi_parallel_algorithm.jl")
include("problems/multi_region_problem.jl")

end
