module PowerSimulationsDecomposition

export MultiRegionProblem
export MultiProblemTemplate

export SplitAreaPTDFPowerModel
export StaticBranchUnboundedStateEstimation
export NetworkFlowConstraintStateEstimation

export get_coordination
export CoordinationAlgorithm
export NoCoordination
export CostCurveCoordination
export ShadowPriceCoordination
# export ADMMCoordination

import PowerSimulations
import PowerNetworkMatrices
import PowerSystems
import InfrastructureSystems
import InfrastructureSystems: @assert_op
import JuMP
import Dates
import MPI
import MathOptInterface
import DataStructures: OrderedDict, SortedDict

const PSI = PowerSimulations
const PSY = PowerSystems
const PNM = PowerNetworkMatrices
const IS = InfrastructureSystems
const ISOPT = InfrastructureSystems.Optimization
const ISSIM = InfrastructureSystems.Simulation
const PM = PSI.PM
const MOI = MathOptInterface

using DocStringExtensions
@template (FUNCTIONS, METHODS) = """
                                 $(TYPEDSIGNATURES)
                                 $(DOCSTRING)
                                 """

include("core/algorithms.jl")
include("core/definitions.jl")
include("core/formulations.jl")
include("core/mpi_info.jl")
include("core/parameters.jl")
include("core/auxiliary_variables.jl")
include("core/constraints.jl")
include("core/coordination_algorithms.jl")
include("multiproblem_template.jl")
include("multi_optimization_container.jl")
include("core/coordination_algorithms_methods.jl")
include("algorithms/sequential_algorithm.jl")
include("algorithms/mpi_parallel_algorithm.jl")
include("algorithms/coordination/no_coordination.jl")
include("algorithms/coordination/cost_curve_coordination.jl")
include("algorithms/coordination/shadow_price_coordination.jl")
include("algorithms/coordination/admm_coordination.jl")

include("problems/multi_region_problem.jl")
include("models/network_models.jl")
include("models/branch_models.jl")
include("print.jl")
end
