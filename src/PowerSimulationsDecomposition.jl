module PowerSimulationsDecomposition

import PowerSimulations
import PowerSystems
import InfrastructureSystems
import JuMP
import Dates
import MPI

const PSI = PowerSimulations
const PSY = PowerSystems
const IS = InfrastructureSystems
const PM = PSI.PM


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
