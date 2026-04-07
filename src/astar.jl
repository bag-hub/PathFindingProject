#**********ALGO Astar******************************************************

function algoAstar(fname::String, D::Tuple{Int64,Int64}, A::Tuple{Int64,Int64})
    env = emptyEnv(fname)
    map = env.map
    pq = PriorityQueue{Tuple{Int64,Int64}, Int64}()

    dist = Dict{Tuple{Int64,Int64},Int64}()          # g(n)
    dictParent = Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}()

    dist[D] = 0
    dictParent[D] = D

    pq[D] = heuristic(D, A)

    #treated permet de stocker les sommets dont leur coût est déjà minimal afin d'optimiser en ne les rajoutant encore dans la file de priorité
    treated = Set{Tuple{Int64,Int64}}()
    #Pour  compter le nombre de sommets visités : mesurer la performance
    visited = Set{Tuple{Int64,Int64}}()

    while !isempty(pq)

        u = dequeue!(pq)

        if u == A
            break
        end

        if !(u in treated)
            push!(treated, u)
            push!(visited, u)

            for v in neighbors(env, u, -1)
                push!(visited, v)
                newDist = dist[u] + map[v[1],v[2]]

                if !haskey(dist, v) || newDist < dist[v]
                    dist[v] = newDist
                    dictParent[v] = u
                    # f(n) = g(n) + h(n)
                    pq[v] = newDist + heuristic(v, A)
                    if !haskey(dist, v)
                        cpt+=1
                    end
                end
            end
        end
    end

    if !haskey(dist, A)
        println("A n'est pas atteignable depuis D")
        return
    end

    coutCh=map[D[1],D[2]]
    chemin = Tuple{Int64,Int64}[]
    push!(chemin, A)
    s = dictParent[A]
    while s != D
        coutCh+=map[s[1],s[2]]
        push!(chemin, s)
        s = dictParent[s]
    end

    push!(chemin, D)
    reverse!(chemin)

    distance = length(chemin)-1
    nbVisited = length(visited)
    println("Distance D->A :", distance)
    println("Number of states evaluated : ", nbVisited)
    println("Coût D->A :", dist[A])
    println("Coût réelle D->A :", coutCh)

    str = string(D)
    #début : fin : pas
    for e in chemin[2:end]
        str *= "->" * string(e)
    end
    println("Path D->A : " * str)
    nbVisited = length(visited)
    return map,chemin,visited,distance,nbVisited,coutCh
end