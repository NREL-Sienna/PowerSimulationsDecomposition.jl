##### PSI methods for TransportHVDCNetworkModel with SplitAreaPTDFPowerModel #######
# The core idea is that the MT-HVDC network is added completely for every subproblem,
# so every problem has complete access to the HVDC network model regardless of the
# area subsystem partitioning applied to the AC network.

"""
Override of PSI.initialize_hvdc_system! for SplitAreaPTDFPowerModel.

For SplitAreaPTDFPowerModel, the HVDC network is shared globally across all area
subsystems. The default PSI implementation filters DC buses by subsystem via
`get_available_components`, which would return an empty set for subproblems whose
subsystem contains no DC buses. This override uses `PSY.get_components` to include
ALL DC buses from the system, ensuring the ActivePowerBalance expression for DCBus
is correctly initialized in every subproblem container.
"""
function PSI.initialize_hvdc_system!(
    container::PSI.OptimizationContainer,
    network_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
    dc_model::PSI.TransportHVDCNetworkModel,
    system::PSY.System,
)
    dc_buses = PSY.get_components(PSY.DCBus, system)
    @assert !isempty(dc_buses) "No DC buses found in the system. \
        Consider adding DC Buses or removing the HVDC network model."
    dc_bus_numbers = sort(PSY.get_number.(dc_buses))
    container.expressions[ISOPT.ExpressionKey(PSI.ActivePowerBalance, PSY.DCBus)] =
        PSI._make_container_array(dc_bus_numbers, PSI.get_time_steps(container))
    return
end

"""
Builds MT-HVDC nodal balance constraints for TransportHVDCNetworkModel when the
network formulation is SplitAreaPTDFPowerModel and the problem is built from a
MultiProblemTemplate.

Every subproblem receives the complete set of DC nodal balance constraints
(NodalBalanceActiveConstraint over all DC buses in the system), providing full
access to the HVDC network model independent of the AC area partitioning.
"""
function PSI.construct_hvdc_network!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    transmission_model::PSI.NetworkModel{SplitAreaPTDFPowerModel},
    hvdc_model::PSI.TransportHVDCNetworkModel,
    ::MultiProblemTemplate,
)
    PSI.add_constraints!(
        container,
        PSI.NodalBalanceActiveConstraint,
        sys,
        transmission_model,
        hvdc_model,
    )
    return
end
