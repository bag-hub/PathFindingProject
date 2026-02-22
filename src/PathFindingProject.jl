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
#Focntions pour la carte
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
    n,m = size(map)
    
    #Initialisation de la file et ajout de D
    q = Queue{Tuple{Int,Int}}()
    enqueue!(q,D)

    b = true # Ce booléen permet de savoir si on a atteint A

    visited = Set{Tuple{Int,Int}}()
    dictParent = Dict{Tuple{Int,Int},Tuple{Int,Int}}(D=>D)
    #Chemin parcouru . NB : L'ajout à la fin du vector se fait en temps constant en amorti
    chemin = Tuple{Int,Int}[]

    #Parcours en largeur du graphe
    while b & !(isempty(q))
         u = dequeue!(q)
         if u==A 
            b = false
            push!(visited,u)
            #dictParent[A] = u
            continue
         end

         if !(u in visited)
            push!(visited,u)
            ngbr = neighbors(map,u)
            for s in ngbr
                ys,xs = s
                if !(s in visited)
                    enqueue!(q,s)
                    dictParent[s] = u
                end
            end
        end
    end

    #Constitution du plus court chemin parcouru de D à A
    push!(chemin,A)
    s = dictParent[A]
    while s!=D
        push!(chemin,s)
        s = dictParent[s]
    end
    push!(chemin,D)
    #reverse se fait en temps constant pour le type Vector
    chemin = reverse!(chemin)
    return chemin
end