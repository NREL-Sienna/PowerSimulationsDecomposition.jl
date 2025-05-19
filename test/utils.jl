
function make_subsystems!(sys, area_subsystem_map)
    subsystems = unique([v for (_, v) in area_subsystem_map])
    for subsystem in subsystems
        add_subsystem!(sys, subsystem)
    end
    for b in get_components(Area, sys)
        area_name = get_name(b)
        add_component_to_subsystem!(sys, area_subsystem_map[area_name], b)
    end
    for b in get_components(Bus, sys)
        area_name = get_name(get_area(b))
        add_component_to_subsystem!(sys, area_subsystem_map[area_name], b)
    end
    for b in get_components(StaticInjection, sys)
        area_name = get_name(get_area(get_bus(b)))
        add_component_to_subsystem!(sys, area_subsystem_map[area_name], b)
    end
end

function add_monitored_lines!(sys, convert_to_monitored_line)
    for line_data in convert_to_monitored_line
        l = get_component(Line, sys, line_data.name)
        convert_component!(sys, l, MonitoredLine)
        l = get_component(MonitoredLine, sys, line_data.name)
        set_flow_limits!(l, (from_to=line_data.flow_limit, to_from=line_data.flow_limit))
    end
end

function add_interchanges!(sys)
    areas = collect(get_components(Area, sys))
    area_names = [get_name(x) for x in areas]
    areas_sorted = areas[sortperm(area_names)]
    for (ix, i) in enumerate(areas_sorted)
        for (jx, j) in enumerate(areas_sorted)
            i_name = get_name(i)
            j_name = get_name(j)
            if i_name <= j_name
                continue
            end
            interchange = AreaInterchange(;
                name=i_name * "_" * j_name,
                available=true,
                active_power_flow=0.0,
                from_area=i,
                to_area=j,
                flow_limits=(from_to=99999, to_from=99999),
            )
            add_component!(sys, interchange)
        end
    end
end
