# Formulations

struct SplitAreaPTDFPowerModel <: PSI.AbstractPTDFModel end

"""
Branch type to avoid flow constraints that uses state estimation flows.
"""
struct StaticBranchUnboundedStateEstimation <: PSI.AbstractBranchFormulation end
