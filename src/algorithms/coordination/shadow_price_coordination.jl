const _SP_RELIEF_PENALTY = 1.0e7
const _SP_LOWER_BOUND_RELAXATION = -1.0e4
const _SP_PRICE_TIGHTEN = 0.00001

const _SP_NETWORK_FLOW_KEY = ISOPT.ConstraintKey(
    PSI.NetworkFlowConstraint, PSY.MonitoredLine, "",
)
const _SP_FLOW_VARIABLE_KEY = ISOPT.VariableKey(
    PSI.FlowActivePowerVariable, PSY.MonitoredLine, "",
)

Base.@kwdef mutable struct ShadowPriceCoordination <: CoordinationAlgorithm
    coordinated_lines::Vector{String}
    coordinated_regions::Vector{String} = String[]
    adjust_rate::Float64 = 0.01
    shadow_price::Dict{Tuple{String, String}, Float64} =
        Dict{Tuple{String, String}, Float64}()
    flow_contribution::Dict{Tuple{String, String}, Float64} =
        Dict{Tuple{String, String}, Float64}()
    prev_flow_contribution::Dict{Tuple{String, String}, Float64} =
        Dict{Tuple{String, String}, Float64}()
    relief_value::Dict{Tuple{String, String}, Float64} =
        Dict{Tuple{String, String}, Float64}()
    step_index::Int = 0
end

_sp_relief_index(line::String) = "$(line)_1"

function _sp_is_coordinated_region(algo::ShadowPriceCoordination, subproblem_id::String)
    isempty(algo.coordinated_regions) && return true
    return subproblem_id in algo.coordinated_regions
end

function _sp_coordinated_lines_in_subproblem(
    algo::ShadowPriceCoordination,
    subproblem::PSI.OptimizationContainer,
)
    haskey(subproblem.variables, _SP_FLOW_VARIABLE_KEY) || return String[]
    available = axes(subproblem.variables[_SP_FLOW_VARIABLE_KEY])[1]
    return [l for l in algo.coordinated_lines if l in available]
end

function initialize_coordination!(
    algo::ShadowPriceCoordination,
    subproblem::PSI.OptimizationContainer,
    sys::PSY.System,
)
    length(PSI.get_time_steps(subproblem)) == 1 || return
    haskey(subproblem.variables, _SP_FLOW_VARIABLE_KEY) || return
    haskey(subproblem.constraints, _SP_NETWORK_FLOW_KEY) || return

    coord_lines = _sp_coordinated_lines_in_subproblem(algo, subproblem)
    isempty(coord_lines) && return

    relief_indices = String[_sp_relief_index(l) for l in coord_lines]

    relief = PSI.add_variable_container!(
        subproblem,
        Relief(),
        PSY.MonitoredLine,
        relief_indices,
        PSI.get_time_steps(subproblem);
        meta = "",
    )

    jump_model = PSI.get_jump_model(subproblem)
    flow_array = subproblem.variables[_SP_FLOW_VARIABLE_KEY]
    net_flow_con_array = subproblem.constraints[_SP_NETWORK_FLOW_KEY]

    for l in coord_lines
        idx = _sp_relief_index(l)
        relief[idx, 1] = JuMP.@variable(jump_model, base_name = "Relief_{1,$(l)}")
        JuMP.set_lower_bound(relief[idx, 1], 0.0)
        JuMP.set_upper_bound(relief[idx, 1], 0.0)
        JuMP.set_objective_coefficient(jump_model, relief[idx, 1], _SP_RELIEF_PENALTY)
    end

    for l in coord_lines
        flow_var = flow_array[l, 1]
        net_flow_con = net_flow_con_array[l, 1]
        JuMP.set_lower_bound(flow_var, _SP_LOWER_BOUND_RELAXATION)
        idx = _sp_relief_index(l)
        JuMP.set_normalized_coefficient(net_flow_con, relief[idx, 1], -1.0)
    end
    return
end

function update_coordination!(
    algo::ShadowPriceCoordination,
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

    local_coord_lines = _sp_coordinated_lines_in_subproblem(algo, subproblem)
    participates = _sp_is_coordinated_region(algo, subproblem_id)

    for l in algo.coordinated_lines
        contrib = 0.0
        for bus_no in bus_numbers
            contrib += ptdf[l, bus_no] * bus_balance_values[bus_no, 1]
        end

        if haskey(algo.flow_contribution, (subproblem_id, l))
            algo.relief_value[(subproblem_id, l)] =
                contrib - algo.flow_contribution[(subproblem_id, l)]
        else
            algo.relief_value[(subproblem_id, l)] = 0.0
        end
        algo.prev_flow_contribution[(subproblem_id, l)] =
            get(algo.flow_contribution, (subproblem_id, l), 0.0)
        algo.flow_contribution[(subproblem_id, l)] = contrib

        if participates && (l in local_coord_lines) &&
           haskey(subproblem.variables, _SP_FLOW_VARIABLE_KEY) &&
           l in axes(subproblem.variables[_SP_FLOW_VARIABLE_KEY])[1]
            flow_var = subproblem.variables[_SP_FLOW_VARIABLE_KEY][l, 1]
            of = JuMP.value(flow_var)
            relief_used = 0.0
            relief_key = ISOPT.VariableKey(Relief, PSY.MonitoredLine, "")
            if haskey(subproblem.variables, relief_key)
                relief_arr = subproblem.variables[relief_key]
                ridx = _sp_relief_index(l)
                if ridx in axes(relief_arr)[1]
                    relief_used = JuMP.value(relief_arr[ridx, 1])
                end
            end

            price = NaN
            jm = PSI.get_jump_model(subproblem)
            cap = PSY.get_rating(PSY.get_component(PSY.MonitoredLine, sys, l))
            fixed_bins = JuMP.VariableRef[]
            for v in JuMP.all_variables(jm)
                if JuMP.is_binary(v)
                    bv = round(JuMP.value(v))
                    JuMP.unset_binary(v)
                    JuMP.fix(v, bv; force = true)
                    push!(fixed_bins, v)
                end
            end
            JuMP.set_upper_bound(flow_var, cap - _SP_PRICE_TIGHTEN*rand())
            pstatus = PSI.solve_impl!(subproblem, sys)
            if pstatus == ISSIM.RunStatus.SUCCESSFULLY_FINALIZED && JuMP.has_upper_bound(flow_var)
                try
                    price = -JuMP.dual(JuMP.UpperBoundRef(flow_var))
                catch
                    price = NaN
                end
            end
            JuMP.set_upper_bound(flow_var, cap)
            for v in fixed_bins
                JuMP.unfix(v)
                JuMP.set_binary(v)
            end

            algo.shadow_price[(subproblem_id, l)] = price
            println("  [sp update id=$subproblem_id line=$l]  cleared_flow=$(round(of;digits=5))  price=$(round(price;digits=5))  relief=$(round(relief_used;digits=5))  contribution=$(round(contrib;digits=5))")
        else
            println("  [sp update id=$subproblem_id line=$l]  (not coordinated)  contribution=$(round(contrib;digits=5))")
        end
    end
    return
end

function apply_coordination!(
    algo::ShadowPriceCoordination,
    container::MultiOptimizationContainer,
    sys::PSY.System,
)
    algo.step_index += 1
    println("\n################  SP STEP $(algo.step_index)  ################")

    region_ids = sort(collect(keys(container.subproblems)))

    for l in algo.coordinated_lines
        sefl = 0.0
        for ((id, ln), v) in algo.flow_contribution
            ln == l && (sefl += v)
        end

        cap = PSY.get_rating(PSY.get_component(PSY.MonitoredLine, sys, l))

        coord_ids = [id for id in region_ids if _sp_is_coordinated_region(algo, id) &&
                     haskey(algo.shadow_price, (id, l))]
        println("--- SP line $l   sefl=$(round(sefl;digits=5))   cap=$(round(cap;digits=4)) ---")
        prices = [(id, get(algo.shadow_price, (id, l), NaN)) for id in coord_ids]
        if !isempty(prices)
            parts = join(["region $(id) price=$(round(p;digits=5))" for (id, p) in prices], "   ")
            valid = [p for (_, p) in prices if !isnan(p)]
            spread = length(valid) >= 2 ? maximum(valid) - minimum(valid) : NaN
            println("  prices:   $parts    (spread = $(round(spread;digits=5)))")
        end

        for id in coord_ids
            subproblem = container.subproblems[id]
            haskey(subproblem.constraints, _SP_NETWORK_FLOW_KEY) || continue
            l in axes(subproblem.constraints[_SP_NETWORK_FLOW_KEY])[1] || continue
            relief_key = ISOPT.VariableKey(Relief, PSY.MonitoredLine, "")
            haskey(subproblem.variables, relief_key) || continue
            relief = subproblem.variables[relief_key][_sp_relief_index(l), 1]

            own_cost = get(algo.shadow_price, (id, l), 0.0)
            other_cost = 0.0
            for other_id in coord_ids
                other_id == id && continue
                other_cost = get(algo.shadow_price, (other_id, l), 0.0)
            end

            own_relief_value = get(algo.relief_value, (id, l), 0.0)
            other_relief_value = 0.0
            for other_id in coord_ids
                other_id == id && continue
                other_relief_value = get(algo.relief_value, (other_id, l), 0.0)
            end

            if sefl <= cap + 0.0001 && sefl >= cap - 0.0001
                if own_cost < other_cost - 0.0001
                    JuMP.set_upper_bound(relief, 0.0)
                    JuMP.set_lower_bound(relief, 0.0)
                elseif own_cost > other_cost + 0.0001
                    JuMP.set_upper_bound(relief, algo.adjust_rate * cap)
                    JuMP.set_lower_bound(relief, 0.0)
                else
                    JuMP.set_upper_bound(relief, 0.0)
                    JuMP.set_lower_bound(relief, 0.0)
                end
            elseif sefl >= cap + 0.0001
                if own_cost < other_cost - 0.0001
                    JuMP.set_upper_bound(relief, 0.0)
                    JuMP.set_lower_bound(relief, 0.0)
                elseif own_cost > other_cost + 0.0001
                    JuMP.set_upper_bound(relief, sefl - cap)
                    JuMP.set_lower_bound(relief, 0.0)
                else
                    JuMP.set_upper_bound(relief, (sefl - cap)/2)
                    JuMP.set_lower_bound(relief, 0.0)
                end
            elseif sefl <= cap - 0.0001
                if own_cost < other_cost - 0.0001
                    JuMP.set_upper_bound(relief, 0.0)
                    JuMP.set_lower_bound(relief, sefl - cap)
                elseif own_cost > other_cost + 0.0001
                    JuMP.set_upper_bound(relief, 0.0)
                    JuMP.set_lower_bound(relief, 0.0)
                else
                    JuMP.set_upper_bound(relief, 0.0)
                    JuMP.set_lower_bound(relief, (sefl - cap)/2)
                end
            end

            JuMP.set_objective_coefficient(PSI.get_jump_model(subproblem), relief, other_cost)

            # model_file = "sp_model_step$(algo.step_index)_$(id)_$(l).lp"
            # JuMP.write_to_file(PSI.get_jump_model(subproblem), model_file)
        end
    end
    return
end