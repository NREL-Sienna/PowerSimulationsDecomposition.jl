const GAEVF = JuMP.GenericAffExpr{Float64, VariableRef}
const GQEVF = JuMP.GenericQuadExpr{Float64, VariableRef}

function get_jump_models(model::PSI.DecisionModel{MultiRegionProblem})
    jump_model_dict = Dict()
    subproblem_keys = keys(problem.internal.container.subproblems)
    for k in subproblem_keys
        jump_model_dict[k] = PSI.get_jump_model(
            IS.Optimization.get_container(PSI.get_internal(model)).subproblems[k],
        )
    end
    return jump_model_dict
end

function moi_tests(
    JuMPmodel,
    vars::Int,
    interval::Int,
    lessthan::Int,
    greaterthan::Int,
    equalto::Int,
    binary::Bool,
)
    @test JuMP.num_variables(JuMPmodel) == vars
    @test JuMP.num_constraints(JuMPmodel, GAEVF, MOI.Interval{Float64}) == interval
    @test JuMP.num_constraints(JuMPmodel, GAEVF, MOI.LessThan{Float64}) == lessthan
    @test JuMP.num_constraints(JuMPmodel, GAEVF, MOI.GreaterThan{Float64}) == greaterthan
    @test JuMP.num_constraints(JuMPmodel, GAEVF, MOI.EqualTo{Float64}) == equalto
    @test ((JuMP.VariableRef, MOI.ZeroOne) in JuMP.list_of_constraint_types(JuMPmodel)) ==
          binary

    return
end

function psi_constraint_test(
    model::DecisionModel,
    constraint_keys::Vector{<:PSI.ConstraintKey},
)
    constraints = PSI.get_constraints(model)
    for con in constraint_keys
        if get(constraints, con, nothing) !== nothing
            @test true
        else
            @error con
            @test false
        end
    end
    return
end
