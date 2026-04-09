##################################################
# A* Space-Time (Pseudo-SIPP)
##################################################
function algoAstar2(
    env::Environment,
    D::Tuple{Int64,Int64},
    A::Tuple{Int64,Int64},
    start_time::Int64
)
    map = env.map

    pq = PriorityQueue{Tuple{Int64,Int64,Int64}, Int64}()
    parent = Dict{Tuple{Int64,Int64,Int64}, Tuple{Int64,Int64,Int64}}()
    dist = Dict{Tuple{Int64,Int64,Int64}, Int64}()

    start_state = (D[1], D[2], start_time)

    parent[start_state] = start_state
    dist[start_state] = start_time
    pq[start_state] = start_time + heuristic(D, A)

    treated = Set{Tuple{Int64,Int64,Int64}}()
    final_state = nothing

    while !isempty(pq)
        u = dequeue!(pq)

        if u in treated
            continue
        end

        push!(treated, u)

        uy, ux, ut = u

        if (uy, ux) == A
            final_state = u
            break
        end

        for v in neighbors(env, (uy, ux), ut)
            vy, vx = v
            move_cost = map[vy, vx]
            arrival = ut + move_cost
            next_t = next_free_time(env, v, arrival)
            
            v_state = (vy, vx, next_t)

            if !haskey(dist, v_state) || next_t < dist[v_state]
                dist[v_state] = next_t
                parent[v_state] = u
                pq[v_state] = next_t + heuristic(v, A)
            end
        end
    end

    if final_state == nothing
        return nothing
    end

    path = Tuple{Int64,Int64}[]
    curr = final_state

    while curr != start_state
        push!(path, (curr[1], curr[2]))
        curr = parent[curr]
    end

    push!(path, D)
    reverse!(path)

    makespan = final_state[3] - start_time

    return map, path, Set(), length(path) - 1, 0, makespan
end