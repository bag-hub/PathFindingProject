#**********ALGO Glouton******************************************************

function algoGlouton(fname::String, D::Tuple{Int64,Int64}, A::Tuple{Int64,Int64})
    map = loadMap(fname)

    if map[A[1],A[2]] == typemax(Int64) || map[D[1],D[2]] == typemax(Int64)
        println("A ou D est inaccessible.")
        return
    end

    pq = PriorityQueue{Tuple{Int64,Int64}, Int64}()
    dictParent = Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}()
    dictParent[D] = D
    pq[D] = heuristic(D, A)
    visited = Set{Tuple{Int64,Int64}}()

    while !isempty(pq)

        u = dequeue!(pq)

        if u == A
            break
        end

        if !(u in visited)
            push!(visited, u)
            for v in neighbors(map, u)
                if !(v in visited)
                    dictParent[v] = u
                    #f(n) = h(n)
                    pq[v] = heuristic(v, A)
                end
            end
        end
    end

    if !haskey(dictParent, A)
        println("A n'est pas atteignable depuis D")
        return
    end

    chemin = Tuple{Int64,Int64}[]
    push!(chemin, A)
    cout=map[D[1],D[2]]
    s = dictParent[A]
    while s != D
        cout+=map[s[1],s[2]]
        push!(chemin, s)
        s = dictParent[s]
    end

    push!(chemin, D)
    reverse!(chemin)

    distance = length(chemin)-1
    nbVisited = length(visited)
    println("Distance D->A :", distance)
    println("Number of states evaluated :",nbVisited)
    println("Coût réelle D->A :", cout)

    str = string(D)
    for e in chemin[2:end]
        str *= "->" * string(e)
    end
    println("Path D->A :" * str)
    return map,chemin,visited,distance,nbVisited,cout
end