##################################################
# A* Space-Time (Pseudo-SIPP)
##################################################

function algoAstar2(env::Environment, D::Tuple{Int64,Int64}, A::Tuple{Int64,Int64})
    map = env.map
    
    # La file de priorité stocke maintenant l'état spatio-temporel : (y, x, t)
    pq = PriorityQueue{Tuple{Int64,Int64,Int64}, Int64}()
    
    # Le parent stocke la provenance d'un état vers un autre : (y, x, t) -> (y, x, t)
    parent = Dict{Tuple{Int64,Int64,Int64}, Tuple{Int64,Int64,Int64}}()

    # État initial au temps t = 0 (heure locale de la mission du robot)
    start_state = (D[1], D[2], 0)
    
    parent[start_state] = start_state
    pq[start_state] = heuristic(D, A)

    # treated stocke désormais l'espace ET le temps : (y, x, t) !
    treated = Set{Tuple{Int64,Int64,Int64}}()
    
    final_state = nothing

    while !isempty(pq)
        u_state = dequeue!(pq)
        uy, ux, ut = u_state

        # Condition d'arrêt : on est sur la case d'arrivée
        if (uy, ux) == A
            final_state = u_state
            break
        end

        if !(u_state in treated)
            push!(treated, u_state)

            for v in neighbors(env, (uy, ux), ut)
                vy, vx = v
                
                # Coût réel du déplacement (gestion du sable)
                move_cost = map[vy, vx]
                expected_arrival = ut + move_cost
                
                # Trouver le prochain intervalle de temps libre
                next_t = next_free_time(env, v, expected_arrival)
                
                # Le nouvel état est la position spatiale + l'heure d'arrivée
                v_state = (vy, vx, next_t)

                # Si on n'a pas encore visité CETTE case à CE temps précis
                if !(v_state in treated) && !haskey(parent, v_state)
                    parent[v_state] = u_state
                    
                    # f(n) = g(n) + h(n) => next_t + heuristique
                    pq[v_state] = next_t + heuristic(v, A)
                end
            end
        end
    end

    if final_state == nothing
        return nothing
    end

    # Reconstruction du chemin (on n'extrait que les coordonnées spatiales pour la compatibilité)
    path = Tuple{Int64,Int64}[]
    curr = final_state
    
    while curr != start_state
        push!(path, (curr[1], curr[2]))
        curr = parent[curr]
    end

    push!(path, D)
    reverse!(path)

    # Le vrai coût final est le temps 't' de l'état d'arrivée
    makespan = final_state[3]
    
    return map, path, Set(), length(path) - 1, 0, makespan
end