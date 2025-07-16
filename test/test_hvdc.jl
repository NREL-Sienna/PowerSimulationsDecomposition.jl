# HVDC spans subsystems but is not modeled (build suceeds)
sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "b")
make_subsystems!(sys, area_subsystem_map)
hvdc = get_component(TwoTerminalHVDCLine, sys, "DC1")
PowerSystems.add_component_to_subsystem!(sys, "a", hvdc)
template_uc2 = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel), ["a", "b"])
problem = DecisionModel(
    MultiRegionProblem,
    template_uc2,
    sys;
    name="UC_Subsystem",
    optimizer=optimizer_with_attributes(HiGHS.Optimizer),
)
build_out = build!(problem; output_dir=mktempdir())
@test build_out == PowerSimulations.ModelBuildStatus.BUILT

# HVDC spans subsystems and is modeled (build fails)
sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "b")
make_subsystems!(sys, area_subsystem_map)
hvdc = get_component(TwoTerminalHVDCLine, sys, "DC1")
PowerSystems.add_component_to_subsystem!(sys, "a", hvdc)
template_uc2 = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel), ["a", "b"])
set_device_model!(template_uc2, TwoTerminalHVDCLine, HVDCTwoTerminalLossless)
problem = DecisionModel(
    MultiRegionProblem,
    template_uc2,
    sys;
    name="UC_Subsystem",
    optimizer=optimizer_with_attributes(HiGHS.Optimizer),
)
build_out = build!(problem; console_level=Logging.AboveMaxLevel, output_dir=mktempdir())
@test build_out == PowerSimulations.ModelBuildStatus.FAILED

# HVDC spans subsystems and is modeled but both terminal buses belong to same subsytem (build suceeds)
sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "b")
make_subsystems!(sys, area_subsystem_map)
hvdc = get_component(TwoTerminalHVDCLine, sys, "DC1")
PowerSystems.add_component_to_subsystem!(sys, "a", hvdc)
PowerSystems.remove_component_from_subsystem!(sys, "b", hvdc.arc.to)
PowerSystems.add_component_to_subsystem!(sys, "a", hvdc.arc.to)
template_uc2 = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel), ["a", "b"])
set_device_model!(template_uc2, TwoTerminalHVDCLine, HVDCTwoTerminalLossless)
problem = DecisionModel(
    MultiRegionProblem,
    template_uc2,
    sys;
    name="UC_Subsystem",
    optimizer=optimizer_with_attributes(HiGHS.Optimizer),
)
build_out = build!(problem; output_dir=mktempdir())
@test build_out == PowerSimulations.ModelBuildStatus.BUILT
