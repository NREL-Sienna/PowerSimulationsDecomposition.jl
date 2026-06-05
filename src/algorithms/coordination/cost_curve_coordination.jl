const _RELIEF_PENALTY = 1.0e7
const _LOWER_BOUND_RELAXATION = -1.0e4
const _INFEASIBLE_COST = 1.0e6
const _NUM_DIRECTIONS = 2
const _PRICE_TIGHTEN = 0.001

const _NETWORK_FLOW_KEY = ISOPT.ConstraintKey(
    PSI.NetworkFlowConstraint, PSY.MonitoredLine, "",
)
const _FLOW_VARIABLE_KEY = ISOPT.VariableKey(
    PSI.FlowActivePowerVariable, PSY.MonitoredLine, "",
)

Base.@kwdef mutable struct CostCurveCoordination <: CoordinationAlgorithm
    coordinated_lines::Vector{String}
    coordinated_regions::Vector{String} = String[]
    num_segments::Int = 41
    check_range_rate::Float64 = 0.1
    cost_curve::Dict{Tuple{String, String}, Vector{Tuple{Float64, Float64}}} =
        Dict{Tuple{String, String}, Vector{Tuple{Float64, Float64}}}()
    flow_contribution::Dict{Tuple{String, String}, Float64} =
        Dict{Tuple{String, String}, Float64}()
    own_flow::Dict{Tuple{String, String}, Float64} =
        Dict{Tuple{String, String}, Float64}()
    relief_used::Dict{Tuple{String, String}, Float64} =
        Dict{Tuple{String, String}, Float64}()
    shadow_price::Dict{Tuple{String, String}, Float64} =
        Dict{Tuple{String, String}, Float64}()
    offset::Dict{Tuple{String, String}, Float64} =
        Dict{Tuple{String, String}, Float64}()
    step_index::Int = 0
end

_relief_index(line::String, seg::Int, dir::Int) = "$(line)_$(seg)_$(dir)"

function _is_coordinated_region(algo::CostCurveCoordination, subproblem_id::String)
    isempty(algo.coordinated_regions) && return true
    return subproblem_id in algo.coordinated_regions
end

function _coordination_N(algo::CostCurveCoordination)
    n = length(algo.coordinated_regions)
    n <= 1 && return 0
    return min(n - 1, _NUM_DIRECTIONS)
end

function _coordinated_lines_in_subproblem(
    algo::CostCurveCoordination,
    subproblem::PSI.OptimizationContainer,
)
    haskey(subproblem.variables, _FLOW_VARIABLE_KEY) || return String[]
    available = axes(subproblem.variables[_FLOW_VARIABLE_KEY])[1]
    return [l for l in algo.coordinated_lines if l in available]
end

function initialize_coordination!(
    algo::CostCurveCoordination,
    subproblem::PSI.OptimizationContainer,
    sys::PSY.System,
)
    length(PSI.get_time_steps(subproblem)) == 1 || return
    haskey(subproblem.variables, _FLOW_VARIABLE_KEY) || return
    haskey(subproblem.constraints, _NETWORK_FLOW_KEY) || return

    coord_lines = _coordinated_lines_in_subproblem(algo, subproblem)
    isempty(coord_lines) && return

    segments = 1:algo.num_segments
    directions = 1:_NUM_DIRECTIONS

    relief_indices = String[
        _relief_index(l, s, d)
        for l in coord_lines, s in segments, d in directions
    ] |> vec

    relief = PSI.add_variable_container!(
        subproblem,
        Relief(),
        PSY.MonitoredLine,
        relief_indices,
        PSI.get_time_steps(subproblem);
        meta = "",
    )

    jump_model = PSI.get_jump_model(subproblem)
    flow_array = subproblem.variables[_FLOW_VARIABLE_KEY]
    net_flow_con_array = subproblem.constraints[_NETWORK_FLOW_KEY]

    for l in coord_lines, s in segments, d in directions
        idx = _relief_index(l, s, d)
        relief[idx, 1] = JuMP.@variable(jump_model, base_name = "Relief_{$(d)_$(s),$(l)}")
        JuMP.set_lower_bound(relief[idx, 1], 0.0)
        JuMP.set_upper_bound(relief[idx, 1], 0.0)
        JuMP.set_objective_coefficient(jump_model, relief[idx, 1], _RELIEF_PENALTY)
    end

    for l in coord_lines
        flow_var = flow_array[l, 1]
        net_flow_con = net_flow_con_array[l, 1]
        JuMP.set_lower_bound(flow_var, _LOWER_BOUND_RELAXATION)
        for s in segments, d in directions
            idx = _relief_index(l, s, d)
            # JuMP.set_normalized_coefficient(net_flow_con, relief[idx, 1], 1.0)
            JuMP.set_normalized_coefficient(net_flow_con, relief[idx, 1], -1.0)
        end
    end
    return
end

function update_coordination!(
    algo::CostCurveCoordination,
    container::MultiOptimizationContainer,
    sys::PSY.System,
    subproblem_id::String,
)
    subproblem = get_subproblem(container, subproblem_id)
    length(PSI.get_time_steps(subproblem)) == 1 || return

    ptdf = PSI.VirtualPTDF(sys; tol = 1e-4, max_cache_size = 10000)
    bus_numbers = get(container.subproblem_bus_map, subproblem_id, Int[])
    isempty(bus_numbers) && return
    bus_balance_expr = PSI.get_expression(subproblem, PSI.ActivePowerBalance(), PSY.ACBus)
    bus_balance_values = JuMP.value.(bus_balance_expr)

    relief_key = ISOPT.VariableKey(Relief, PSY.MonitoredLine, "")
    relief = haskey(subproblem.variables, relief_key) ?
        subproblem.variables[relief_key] : nothing

    local_coord_lines = _coordinated_lines_in_subproblem(algo, subproblem)
    participates = _is_coordinated_region(algo, subproblem_id)

    for l in algo.coordinated_lines
        contrib = 0.0
        for bus_no in bus_numbers
            contrib += ptdf[l, bus_no] * bus_balance_values[bus_no, 1]
        end
        algo.flow_contribution[(subproblem_id, l)] = contrib

        if participates && (l in local_coord_lines)
            flow_var = subproblem.variables[_FLOW_VARIABLE_KEY][l, 1]
            of = JuMP.value(flow_var)
            algo.own_flow[(subproblem_id, l)] = of

            ru = 0.0
            if relief !== nothing
                for d in 1:_NUM_DIRECTIONS, s in 1:algo.num_segments
                    idx = _relief_index(l, s, d)
                    idx in axes(relief)[1] || continue
                    ru += JuMP.value(relief[idx, 1])
                end
            end
            algo.relief_used[(subproblem_id, l)] = ru

            price = NaN
            if haskey(subproblem.constraints, _NETWORK_FLOW_KEY) &&
               l in axes(subproblem.constraints[_NETWORK_FLOW_KEY])[1]
                con = subproblem.constraints[_NETWORK_FLOW_KEY][l, 1]
                jm = PSI.get_jump_model(subproblem)
                flow_var_p = subproblem.variables[_FLOW_VARIABLE_KEY][l, 1]
                orig_ub = JuMP.has_upper_bound(flow_var_p) ? JuMP.upper_bound(flow_var_p) : Inf
                fixed_bins = JuMP.VariableRef[]
                for v in JuMP.all_variables(jm)
                    if JuMP.is_binary(v)
                        bv = round(JuMP.value(v))
                        JuMP.unset_binary(v)
                        JuMP.fix(v, bv; force = true)
                        push!(fixed_bins, v)
                    end
                end
                JuMP.set_upper_bound(flow_var_p, of - _PRICE_TIGHTEN)
                pstatus = PSI.solve_impl!(subproblem, sys)
                if pstatus == ISSIM.RunStatus.SUCCESSFULLY_FINALIZED
                    price = -JuMP.dual(con)
                end
                if isfinite(orig_ub)
                    JuMP.set_upper_bound(flow_var_p, orig_ub)
                end
                for v in fixed_bins
                    JuMP.unfix(v)
                    JuMP.set_binary(v)
                end
            end
            println("  [update id=$subproblem_id line=$l]  cleared_flow=$(round(of;digits=5))  contribution=$(round(contrib;digits=5))  relief_used=$(round(ru;digits=5))  price=$(round(price;digits=5))")

            if relief !== nothing
                for d in 1:_NUM_DIRECTIONS
                    seg_used = Tuple{Int,Float64,Float64,Float64}[]
                    for s in 1:algo.num_segments
                        idx = _relief_index(l, s, d)
                        idx in axes(relief)[1] || continue
                        var = relief[idx, 1]
                        ub = JuMP.has_upper_bound(var) ? JuMP.upper_bound(var) : Inf
                        cost = JuMP.objective_function(PSI.get_jump_model(subproblem)) isa JuMP.GenericAffExpr ?
                            JuMP.coefficient(JuMP.objective_function(PSI.get_jump_model(subproblem)), var) : NaN
                        val = JuMP.value(var)
                        if ub > 1e-9 || val > 1e-9
                            push!(seg_used, (s, ub, cost, val))
                        end
                    end
                    isempty(seg_used) && continue
                    println("      [relief segments dir=$d line=$l]  (segment: width=ub, price=cost, taken=value)")
                    for (s, ub, cost, val) in seg_used
                        filled = ub > 1e-9 ? round(100 * val / ub; digits=1) : 0.0
                        println("        seg $s: width=$(round(ub;digits=5))  price=$(round(cost;digits=2))  taken=$(round(val;digits=5))  ($(filled)% full)")
                    end
                end
            end
        else
            reason = participates ? "(no flow var)" : "(region not coordinated)"
            println("  [update id=$subproblem_id line=$l]  $reason  contribution=$(round(contrib;digits=5))")
        end
    end
    return
end

function apply_coordination!(
    algo::CostCurveCoordination,
    container::MultiOptimizationContainer,
    sys::PSY.System,
)
    algo.step_index += 1
    println("\n################  STEP $(algo.step_index)  ################")
    # if !isempty(algo.coordinated_regions)
    #     println("  coordinated_regions = $(algo.coordinated_regions)   N = $(_coordination_N(algo))")
    # end

    println("--- cleared dispatch (all regions solved) ---")
    region_ids = sort(collect(keys(container.subproblems)))
    for l in algo.coordinated_lines
        for id in region_ids
            haskey(algo.flow_contribution, (id, l)) || continue
            contrib = algo.flow_contribution[(id, l)]
            if haskey(algo.own_flow, (id, l))
                of = algo.own_flow[(id, l)]
                ru = get(algo.relief_used, (id, l), 0.0)
                println("  region $id  cleared_flow=$(round(of;digits=5))  contribution=$(round(contrib;digits=5))  relief_used=$(round(ru;digits=5))")
            else
                tag = _is_coordinated_region(algo, id) ? "(no coordination)" : "(region not coordinated)"
                println("  region $id  $tag  contribution=$(round(contrib;digits=5))")
            end
        end
        contribs = [v for ((r, ln), v) in algo.flow_contribution if ln == l]
        if !isempty(contribs)
            total = sum(contribs)
            cap = PSY.get_rating(PSY.get_component(PSY.MonitoredLine, sys, l))
            # println("  >>> total actual flow on $l = $(round(total;digits=5))   (capacity = $(round(cap;digits=4)))")
        end
    end

    println("--- curve generation ---")
    for (id, subproblem) in container.subproblems
        length(PSI.get_time_steps(subproblem)) == 1 || continue
        _is_coordinated_region(algo, id) || continue
        coord_lines = _coordinated_lines_in_subproblem(algo, subproblem)
        isempty(coord_lines) && continue
        _generate_cost_curves!(algo, subproblem, sys, id, coord_lines)
    end

    println("--- shadow prices at current flow (NetworkFlowConstraint dual, per clearing region) ---")
    for l in algo.coordinated_lines
        sps = [(r, sp) for ((ln, r), sp) in algo.shadow_price if ln == l]
        if !isempty(sps)
            parts = join(["region $(r) (cleared) price=$(round(sp;digits=2))" for (r, sp) in sort(sps)], "   ")
            spread = maximum(sp for (_, sp) in sps) - minimum(sp for (_, sp) in sps)
            println("  line $l:   $parts")
        end
    end

    for (id, subproblem) in container.subproblems
        length(PSI.get_time_steps(subproblem)) == 1 || continue
        _is_coordinated_region(algo, id) || continue
        coord_lines = _coordinated_lines_in_subproblem(algo, subproblem)
        isempty(coord_lines) && continue
        _apply_cost_curves!(algo, subproblem, sys, id, coord_lines)
    end
    return
end

function _shift_constraint_constant!(con::JuMP.ConstraintRef, delta::Float64)
    old = JuMP.normalized_rhs(con)
    JuMP.set_normalized_rhs(con, old + delta)
    return
end

function _generate_cost_curves!(
    algo::CostCurveCoordination,
    subproblem::PSI.OptimizationContainer,
    sys::PSY.System,
    subproblem_id::String,
    coord_lines::Vector{String},
)
    relief = subproblem.variables[ISOPT.VariableKey(Relief, PSY.MonitoredLine, "")]
    net_flow_con_array = subproblem.constraints[_NETWORK_FLOW_KEY]
    segments = 1:algo.num_segments
    directions = 1:_NUM_DIRECTIONS

    for l in coord_lines
        net_flow_con = net_flow_con_array[l, 1]
        for s in segments, d in directions
            idx = _relief_index(l, s, d)
            JuMP.set_normalized_coefficient(net_flow_con, relief[idx, 1], 0.0)
        end
        off = get(algo.offset, (subproblem_id, l), 0.0)
        if off != 0.0
            _shift_constraint_constant!(net_flow_con, +off)
        end
    end

    for l in coord_lines
        algo.cost_curve[(l, subproblem_id)] = _scan_line_curve(algo, subproblem, sys, l, subproblem_id)
    end

    for l in coord_lines
        net_flow_con = net_flow_con_array[l, 1]
        for s in segments, d in directions
            idx = _relief_index(l, s, d)
            JuMP.set_normalized_coefficient(net_flow_con, relief[idx, 1], -1.0)
        end
        off = get(algo.offset, (subproblem_id, l), 0.0)
        if off != 0.0
            _shift_constraint_constant!(net_flow_con, -off)
        end
    end
    return
end

function _scan_line_curve(
    algo::CostCurveCoordination,
    subproblem::PSI.OptimizationContainer,
    sys::PSY.System,
    line_name::String,
    subproblem_id::String,
)
    line = PSY.get_component(PSY.MonitoredLine, sys, line_name)
    capacity = PSY.get_rating(line)
    check_range = capacity * algo.check_range_rate
    step = check_range / ((algo.num_segments - 1) / 2)

    flow_var = subproblem.variables[_FLOW_VARIABLE_KEY][line_name, 1]
    relief_key = ISOPT.VariableKey(Relief, PSY.MonitoredLine, "")
    # relief_sum = 0.0
    # if haskey(subproblem.variables, relief_key)
    #     relief = subproblem.variables[relief_key]
    #     for d in 1:_NUM_DIRECTIONS, s in 1:algo.num_segments
    #         idx = _relief_index(line_name, s, d)
    #         idx in axes(relief)[1] || continue
    #         relief_sum += JuMP.value(relief[idx, 1])
    #     end
    # end
    # off_cur = get(algo.offset, (subproblem_id, line_name), 0.0)
    # current_flow = JuMP.value(flow_var) - relief_sum + off_cur
    # println("generate curve current flow is $(current_flow) ",relief_sum," ",off_cur)
    current_flow = JuMP.value(flow_var)
    original_upper_bound = JuMP.upper_bound(flow_var)

    jump_model = PSI.get_jump_model(subproblem)
    fixed_binaries = JuMP.VariableRef[]
    for v in JuMP.all_variables(jump_model)
        if JuMP.is_binary(v)
            val = JuMP.value(v)
            JuMP.unset_binary(v)
            JuMP.fix(v, round(val); force = true)
            push!(fixed_binaries, v)
        end
    end

    raw = Tuple{Float64, Float64, Float64}[]
    current_cost = NaN
    current_shadow = 0.0
    net_flow_con = subproblem.constraints[_NETWORK_FLOW_KEY][line_name, 1]
    for diff in (-check_range):step:check_range
        new_ub = current_flow - diff
        JuMP.set_upper_bound(flow_var, new_ub)
        status = PSI.solve_impl!(subproblem, sys)
        if status == ISSIM.RunStatus.SUCCESSFULLY_FINALIZED
            price = -JuMP.dual(net_flow_con)
            objv = JuMP.objective_value(jump_model)
        else
            price = _INFEASIBLE_COST
            objv = NaN
        end
        push!(raw, (Float64(diff), price, objv))
        # println("diff is $(diff) $(price) $(objv)")
        # if abs(diff) < step / 2
        #     current_cost = objv
        #     current_shadow = price
        # end
    end

    for v in fixed_binaries
        JuMP.unfix(v)
        JuMP.set_binary(v)
    end
    JuMP.set_upper_bound(flow_var, original_upper_bound)

    algo.shadow_price[(line_name, subproblem_id)] = current_shadow

    points = [(x, y) for (x, y, _) in raw]
    deduped = _deduplicate_curve(points, algo.num_segments)

    off_dbg = get(algo.offset, (subproblem_id, line_name), 0.0)
    # println("  region $subproblem_id  cleared_flow=$(round(current_flow;digits=5))  shadow_price@current=$(round(current_shadow;digits=2))  (offset removed=$(round(off_dbg;digits=5)))")
    # println("      curve (Δflow_reduction, marginal_price, total_cost, Δcost_vs_current):")
    # for (x, y) in deduped
    #     objv = NaN
    #     for (dx, _, dobj) in raw
    #         if abs(dx - x) < 1e-9
    #             objv = dobj
    #             break
    #         end
    #     end
    #     dcost = (isnan(objv) || isnan(current_cost)) ? NaN : objv - current_cost
    #     println("      Δ=$(round(x;digits=5))  price=$(round(y;digits=2))  total_cost=$(round(objv;digits=2))  Δcost=$(round(dcost;digits=2))")
    # end
    return deduped
end

function _deduplicate_curve(
    points::Vector{Tuple{Float64, Float64}},
    max_segments::Int,
)
    sorted = sort(points; by = first)
    seen = Set{Float64}()
    curve = Tuple{Float64, Float64}[]
    for (x, y) in sorted
        y_norm = abs(y) < 1.0e-3 ? 0.0 : round(y; digits = 3)
        if !(y_norm in seen) && length(curve) < max_segments
            push!(seen, y_norm)
            push!(curve, (x, y))
        end
    end
    return curve
end

function _apply_cost_curves!(
    algo::CostCurveCoordination,
    subproblem::PSI.OptimizationContainer,
    sys::PSY.System,
    subproblem_id::String,
    coord_lines::Vector{String},
)
    jump_model = PSI.get_jump_model(subproblem)
    relief = subproblem.variables[ISOPT.VariableKey(Relief, PSY.MonitoredLine, "")]
    net_flow_con_array = subproblem.constraints[_NETWORK_FLOW_KEY]
    segments = 1:algo.num_segments

    for l in coord_lines
        line = PSY.get_component(PSY.MonitoredLine, sys, l)
        check_range = PSY.get_rating(line) * algo.check_range_rate

        other_curves_with_id = [
            (other_id, copy(curve))
            for ((ln, other_id), curve) in algo.cost_curve
            if ln == l && other_id != subproblem_id &&
               _is_coordinated_region(algo, other_id)
        ]

        other_curves = [c for (_, c) in other_curves_with_id]
        for curve in other_curves
            push!(curve, (check_range, curve[end][2]))
        end

        if length(other_curves) > _NUM_DIRECTIONS
            @warn "Line $l has $(length(other_curves)) contributing subproblems; \
                   only the first $_NUM_DIRECTIONS will be applied."
        end

        for (d, (source_id, curve)) in enumerate(other_curves_with_id)
            d > _NUM_DIRECTIONS && break
            _apply_curve_to_relief!(jump_model, relief, l, d, curve, segments, check_range)
            cum = 0.0
            println("  [relief curve  receiver=$subproblem_id  generated_by=$source_id  line=$l  dir=$d]  (each segment starts at 0, width, price, cumulative reduction)")
            num_pieces = length(curve)
            for s in segments
                width = if s < num_pieces
                    curve[s + 1][1] - curve[s][1]
                elseif s == 1 && num_pieces == 1
                    check_range
                else
                    0.0
                end
                width <= 1e-9 && continue
                price = s <= num_pieces ? curve[s][2] : _INFEASIBLE_COST
                cum += width
                println("        seg $s: width=$(round(width;digits=5))  price=$(round(price;digits=2))  cumulative=$(round(cum;digits=5))")
            end
        end

        N = _coordination_N(algo)
        net_flow_con = net_flow_con_array[l, 1]
        old_off = get(algo.offset, (subproblem_id, l), 0.0)
        new_off = N * check_range
        if new_off != old_off
            _shift_constraint_constant!(net_flow_con, -(new_off - old_off))
        end
        algo.offset[(subproblem_id, l)] = new_off
    end
    return
end

function _apply_curve_to_relief!(
    jump_model::JuMP.Model,
    relief::JuMP.Containers.DenseAxisArray,
    line_name::String,
    direction::Int,
    curve::Vector{Tuple{Float64, Float64}},
    segments::UnitRange{Int},
    check_range::Float64,
)
    num_pieces = length(curve)
    for s in segments
        idx = _relief_index(line_name, s, direction)
        var = relief[idx, 1]
        upper = if s < num_pieces
            curve[s + 1][1] - curve[s][1]
        elseif s == 1 && num_pieces == 1
            check_range
        else
            0.0
        end
        JuMP.set_upper_bound(var, upper)
        coeff = s <= num_pieces ? curve[s][2] : _INFEASIBLE_COST
        JuMP.set_objective_coefficient(jump_model, var, coeff)
    end
    return
end