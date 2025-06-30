@testset "Test adding reserves to sub-problems" begin
    template = MultiProblemTemplate(
        NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true),
        ["a", "b"],
    )
    service_model = ServiceModel(VariableReserve{ReserveUp}, RangeReserve, "test")
    set_service_model!(template, service_model)
    @test !isempty(template.sub_templates["a"].services)
    @test !isempty(template.sub_templates["b"].services)

    template = MultiProblemTemplate(
        NetworkModel(SplitAreaPTDFPowerModel; use_slacks=true),
        ["a", "b"],
    )
    set_service_model!(template, VariableReserve{ReserveUp}, RangeReserve)
    @test !isempty(template.sub_templates["a"].services)
    @test !isempty(template.sub_templates["b"].services)
end

@testset "MOI test - w/out reserves" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "b")
    make_subsystems!(sys, area_subsystem_map)
    template = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel), ["a", "b"])
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_service_model!(template, VariableReserve{ReserveUp}, RangeReserve)
    problem = DecisionModel(
        MultiRegionProblem,
        template,
        sys;
        name="UC_Subsystem",
        optimizer=optimizer_with_attributes(HiGHS.Optimizer),
    )
    build_out = build!(problem; output_dir=mktempdir())
    @test build_out == PowerSimulations.ModelBuildStatus.BUILT
    jump_problem_dict = get_jump_models(problem)
    moi_tests(jump_problem_dict["a"], 9282, 0, 1728, 864, 2640, true)
    moi_tests(jump_problem_dict["b"], 15012, 0, 3456, 1728, 5280, true)
end

@testset "MOI test - reserves in A" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    for (k, v) in get_contributing_device_mapping(sys)
        s = v.service
        if get_name(s) ∈ ["Reg_Up", "Reg_Down"]
            vec_d = v.contributing_devices
            for d in vec_d
                remove_service!(d, s)
            end
        end
    end
    area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "b")
    make_subsystems!(sys, area_subsystem_map)
    r1 = get_component(VariableReserve, sys, "Spin_Up_R1")
    add_component_to_subsystem!(sys, "a", r1)
    template = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel), ["a", "b"])
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_service_model!(template, VariableReserve{ReserveUp}, RangeReserve)
    problem = DecisionModel(
        MultiRegionProblem,
        template,
        sys;
        name="UC_Subsystem",
        optimizer=optimizer_with_attributes(HiGHS.Optimizer),
    )
    build_out = build!(problem; output_dir=mktempdir())
    @test build_out == PowerSimulations.ModelBuildStatus.BUILT
    jump_problem_dict = get_jump_models(problem)
    moi_tests(jump_problem_dict["a"], 10098, 0, 1728, 912, 2640, true)
    moi_tests(jump_problem_dict["b"], 15012, 0, 3456, 1728, 5280, true)
end

@testset "MOI test - reserves in B" begin
    sys = build_system(PSISystems, "modified_RTS_GMLC_DA_sys")
    for (k, v) in get_contributing_device_mapping(sys)
        s = v.service
        if get_name(s) ∈ ["Reg_Up", "Reg_Down"]
            vec_d = v.contributing_devices
            for d in vec_d
                remove_service!(d, s)
            end
        end
    end
    area_subsystem_map = Dict("1" => "a", "2" => "b", "3" => "b")
    make_subsystems!(sys, area_subsystem_map)
    r2 = get_component(VariableReserve, sys, "Spin_Up_R2")
    add_component_to_subsystem!(sys, "b", r2)
    template = MultiProblemTemplate(NetworkModel(SplitAreaPTDFPowerModel), ["a", "b"])
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_service_model!(template, VariableReserve{ReserveUp}, RangeReserve)
    problem = DecisionModel(
        MultiRegionProblem,
        template,
        sys;
        name="UC_Subsystem",
        optimizer=optimizer_with_attributes(HiGHS.Optimizer),
    )
    build_out = build!(problem; output_dir=mktempdir())
    @test build_out == PowerSimulations.ModelBuildStatus.BUILT
    jump_problem_dict = get_jump_models(problem)
    moi_tests(jump_problem_dict["a"], 9282, 0, 1728, 864, 2640, true)
    moi_tests(jump_problem_dict["b"], 15924, 0, 3456, 1776, 5280, true)
end
