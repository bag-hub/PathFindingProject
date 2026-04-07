#**********ALGO Dikstra******************************************************

function algoDijkstra(fname::String, D::Tuple{Int64,Int64}, A::Tuple{Int64,Int64})
    env = emptyEnv(fname)
    map = env.map

    #Cas où A ou D se trouve sur un obsltacle : un mur ou un arbre
    if (map[A[1],A[2]]==typemax(Int64) || map[D[1],D[2]]==typemax(Int64))
        println("A ou D est inacessible.")
    end

    # File de priorité (clé = sommet, priorité = distance)
    pq = PriorityQueue{Tuple{Int64,Int64}, Int64}()

    # Dictionnaire de distance minimale connue et dictionnaire du parent pour l'arbre issu du parcourt
    dist = Dict{Tuple{Int64,Int64},Int64}()
    dictParent = Dict{Tuple{Int64,Int64}, Tuple{Int64,Int64}}()

    # Initialisation
    dist[D] = 0
    dictParent[D] = D
    pq[D] = 0

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

            for v in neighbors(env, u,-1)
                push!(visited, v)
                newDist = dist[u] + map[v[1], v[2]]
                 #Ici on utilise l'évaluation naïve des expressions booléen pour l'opérateur '||'
                if !haskey(dist, v) || newDist < dist[v]
                    dist[v] = newDist
                    dictParent[v] = u
                    #f(n) = g(n)
                    pq[v] = newDist
                end
            end
        end
    end

    #Dans le cas où A est inacessible à partir de D
    if !haskey(dictParent,A)
        println("A n'est pas atteignable depuis D")
        return
    end

    # Reconstruction du chemin et calcul coût
    cout=map[D[1],D[2]]
    chemin = Tuple{Int64,Int64}[]
    push!(chemin, A)
    s = dictParent[A]
    while s != D
        cout+=map[s[1],s[2]]
        push!(chemin, s)
        s = dictParent[s]
    end

    push!(chemin, D)
    chemin = reverse!(chemin)

    distance = length(chemin)-1
    nbVisited = length(visited)
    println("Distance D->A : ", )
    println("Number of states evaluated : ", nbVisited)
    println("Coût D->A :", dist[A])
    println("Coût réelle D->A : ", cout)

    str = string(D)
    for e in chemin[2:end]
        str = str * "->" * string(e)
    end
    println("Path D->A : " * str)

    return map,chemin,visited,distance,nbVisited,cout
end