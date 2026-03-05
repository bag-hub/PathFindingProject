#=module PathFindingProject
using DataStructures

include("grid.jl")
include("utils.jl")
include("bfs.jl")
include("dijkstra.jl")
include("glouton.jl")
include("astar.jl")

export algoBFS, algoDijkstra, algoGlouton, algoAstar,loadMap

end=#

using DataStructures

#**************************************************
#Fonction pour charger une carte et créer la matrice correspondant à  cette carte
function loadMap(fname::String)
    #Lecture fichier
    lines = open(fname,"r") do f
        readlines(f)
    end

    #Les 4 premières lignes représentent les métadonnées
    n = length(lines)-4 #Nombres de lignes 
    m = length(lines[5]) #Nombres de colonnes
    carte = Matrix{Int}(undef,n,m)
    for i in 1:n
        car = collect(lines[i+4])
        for j in 1:m
            c = car[j]
            if c=='.'
                carte[i,j] = 1
            elseif c=='S'
                carte[i,j] = 5
            elseif c=='W'
                carte[i,j] = 8
            else
                carte[i,j] = typemax(Int)
            end
        end
    end
    return carte
end

function neighbors(grille::Matrix{Int},s::Tuple{Int,Int})
    y,x = s
    n,m = size(grille)
    res = MutableLinkedList{Tuple{Int,Int}}()
    
    #Droite
    if x<m
        if grille[y,x+1]!= typemax(Int)
            push!(res,(y,x+1))
        end
    end
    
    #Gauche
    if x>1 
        if grille[y,x-1]!= typemax(Int)
            push!(res,(y,x-1))
        end
    end

    #Haut
    if y>1 
        if grille[y-1,x]!= typemax(Int)
            push!(res,(y-1,x))
        end
    end

    #Bas
    if y<n
        if grille[y+1,x]!= typemax(Int)
            push!(res,(y+1,x))
        end
    end

    return res
end


#***********ALGO BFS*********************************

function algoBFS(fname::String, D::Tuple{Int,Int}, A::Tuple{Int,Int})
    map = loadMap(fname)

    #Cas où A ou D se trouve sur un obsltacle : un mur ou un arbre
    if (map[A[1],A[2]]==typemax(Int) || map[A[1],A[2]]==typemax(Int))
        println("A ou D est inacessible.")
    end

    #Initialisation de la file et ajout de D
    q = Queue{Tuple{Int,Int}}()
    enqueue!(q,D)

    visited = Set{Tuple{Int,Int}}()
    dictParent = Dict{Tuple{Int,Int},Tuple{Int,Int}}(D=>D)
    #Chemin parcouru . NB : L'ajout à la fin du vector se fait en temps constant en amorti
    chemin = Tuple{Int,Int}[]

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
    println("Distance D->A :",length(chemin))
    println("Number of states evaluated :",length(visited))
    println("Coût D->A :", cout)
    println("Coût réelle D->A :", cout)
    str = string(D)
    for e in  chemin[2:end]
        str=str*"->"*string(e)
    end
    println("Path D->A : "*str)
end

#**********ALGO Dikstra******************************************************

function algoDijkstra(fname::String, D::Tuple{Int,Int}, A::Tuple{Int,Int})
    map = loadMap(fname)

    #Cas où A ou D se trouve sur un obsltacle : un mur ou un arbre
    if (map[A[1],A[2]]==typemax(Int) || map[D[1],D[2]]==typemax(Int))
        println("A ou D est inacessible.")
    end

    # File de priorité (clé = sommet, priorité = distance)
    pq = PriorityQueue{Tuple{Int,Int}, Int}()

    # Dictionnaire de distance minimale connue et dictionnaire du parent pour l'arbre issu du parcourt
    dist = Dict{Tuple{Int,Int},Int}()
    dictParent = Dict{Tuple{Int,Int}, Tuple{Int,Int}}()

    # Initialisation
    dist[D] = 0
    dictParent[D] = D
    pq[D] = 0

    #treated permet de stocker les sommets dont leur coût est déjà minimal afin d'optimiser en ne les rajoutant encore dans la file de priorité
    treated = Set{Tuple{Int,Int}}()
    #Pour  compter le nombre de sommets visités : mesurer la performance
    visited = Set{Tuple{Int,Int}}()

    while !isempty(pq)

        u = dequeue!(pq)

        if u == A
            break
        end

        if !(u in treated)
            push!(treated, u)
            push!(visited, u)

            for v in neighbors(map, u)
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
    chemin = Tuple{Int,Int}[]
    push!(chemin, A)
    s = dictParent[A]
    while s != D
        cout+=map[s[1],s[2]]
        push!(chemin, s)
        s = dictParent[s]
    end

    push!(chemin, D)
    chemin = reverse!(chemin)

    println("Distance D->A : ", length(chemin))
    println("Number of states evaluated : ", length(visited))
    println("Coût D->A :", dist[A])
    println("Coût réelle D->A : ", cout)

    str = string(D)
    for e in chemin[2:end]
        str = str * "->" * string(e)
    end
    println("Path D->A : " * str)
end



#**********ALGO Astar******************************************************

function heuristic(u::Tuple{Int,Int}, A::Tuple{Int,Int})
    return abs(u[1] - A[1]) + abs(u[2] - A[2])
end


function algoAstar(fname::String, D::Tuple{Int,Int}, A::Tuple{Int,Int})
    map = loadMap(fname)

    pq = PriorityQueue{Tuple{Int,Int}, Int}()

    dist = Dict{Tuple{Int,Int},Int}()          # g(n)
    dictParent = Dict{Tuple{Int,Int},Tuple{Int,Int}}()

    dist[D] = 0
    dictParent[D] = D

    pq[D] = heuristic(D, A)

    #treated permet de stocker les sommets dont leur coût est déjà minimal afin d'optimiser en ne les rajoutant encore dans la file de priorité
    treated = Set{Tuple{Int,Int}}()
    #Pour  compter le nombre de sommets visités : mesurer la performance
    visited = Set{Tuple{Int,Int}}()

    while !isempty(pq)

        u = dequeue!(pq)

        if u == A
            break
        end

        if !(u in treated)
            push!(treated, u)
            push!(visited, u)

            for v in neighbors(map, u)
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
    chemin = Tuple{Int,Int}[]
    push!(chemin, A)
    s = dictParent[A]
    while s != D
        coutCh+=map[s[1],s[2]]
        push!(chemin, s)
        s = dictParent[s]
    end

    push!(chemin, D)
    reverse!(chemin)

    println("Distance D->A :", length(chemin))
    println("Number of states evaluated : ", length(visited))
    println("Coût D->A :", dist[A])
    println("Coût réelle D->A :", coutCh)

    str = string(D)
    #début : fin : pas
    for e in chemin[2:end]
        str *= "->" * string(e)
    end
    println("Path D->A : " * str)
end



#**********ALGO Glouton******************************************************

function algoGlouton(fname::String, D::Tuple{Int,Int}, A::Tuple{Int,Int})
    map = loadMap(fname)

    if map[A[1],A[2]] == typemax(Int) || map[D[1],D[2]] == typemax(Int)
        println("A ou D est inaccessible.")
        return
    end

    pq = PriorityQueue{Tuple{Int,Int}, Int}()
    dictParent = Dict{Tuple{Int,Int},Tuple{Int,Int}}()
    dictParent[D] = D
    pq[D] = heuristic(D, A)
    visited = Set{Tuple{Int,Int}}()

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

    chemin = Tuple{Int,Int}[]
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

    println("Distance D->A :", length(chemin))
    println("Number of states evaluated :", length(visited))
    println("Coût réelle D->A :", cout)

    str = string(D)
    for e in chemin[2:end]
        str *= "->" * string(e)
    end
    println("Path D->A :" * str)
end

#= Exemple illlustratif que AlgoGlouton donne pas forcément la meilleure solution : le chemin plus court optimal
include("src/PathFindingProject.jl")
algoBFS("test/exempleMapGlouton.map",(1,1),(10,10))
map = algoDijkstra("test/exempleMapGlouton.map",(1,1),(10,10))
map = algoAstar("test/exempleMapGlouton.map",(1,1),(10,10))
map = algoGlouton("test/exempleMapGlouton.map",(1,1),(10,10))
=#