const CONTAINER_FIELDS = [:variables, :aux_variables, :constraints, :expressions, :duals]
const ALL_CONTAINER_FIELDS =
    [:variables, :aux_variables, :constraints, :expressions, :duals, :parameters]

const CONCRETE_HVDC_TYPES = [:TwoTerminalGenericHVDCLine, :TwoTerminalLCCLine, :TwoTerminalGenericHVDCLine]