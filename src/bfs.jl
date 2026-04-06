
using DataStructures

#***********ALGO BFS*********************************

function algoBFS(fname::String, D::Tuple{Int64,Int64}, A::Tuple{Int64,Int64})
    map = loadMap(fname)

    #Cas où A ou D se trouve sur un obsltacle : un mur ou un arbre
    if (map[A[1],A[2]]==typemax(Int64) || map[A[1],A[2]]==typemax(Int64))
        println("A ou D est inacessible.")
    end

    #Initialisation de la file et ajout de D
    q = Queue{Tuple{Int64,Int64}}()
    enqueue!(q,D)

    visited = Set{Tuple{Int64,Int64}}()
    dictParent = Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}(D=>D)
    #Chemin parcouru . NB : L'ajout à la fin du vector se fait en temps constant en amorti
    chemin = Tuple{Int64,Int64}[]

    #Parcours en largeur du graphe
    while !(isempty(q))
         u = dequeue!(q)
         if u==A 
            push!(visited,u)
            break
         end

         if !(u in visited)
            push!(visited,u)
            ngbr = neighbors(map,u)
            for s in ngbr
                if !(s in visited)
                    enqueue!(q,s)
                    dictParent[s] = u
                end
            end
        end
    end

    #Dans le cas où A est inacessible à partir de D
    if !haskey(dictParent,A)
        println("A n'est pas atteignable depuis D")
        return
    end

    #Constitution du plus court chemin parcouru de D à A et évaluation du coût
    cout=map[D[1],D[2]]
    push!(chemin,A)
    s = dictParent[A]
    while s!=D
        cout+=map[s[1],s[2]]
        push!(chemin,s)
        s = dictParent[s]
    end
    push!(chemin,D)
    #reverse se fait en temps constant pour le type Vector
    chemin = reverse!(chemin)

    distance = length(chemin)-1
    nbVisited = length(visited)
    println("Distance D->A :",distance)
    println("Number of states evaluated :",nbVisited)
    println("Coût D->A :", cout)
    println("Coût réelle D->A :", cout)
    str = string(D)
    for e in  chemin[2:end]
        str=str*"->"*string(e)
    end
    println("Path D->A : "*str)
    return map,chemin,visited,distance,nbVisited,cout
end