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
    res = Tuple{Int,Int}[]
    
    #Droite
    if y<n
        if grille[y+1,x]!= typemax(Int)
            push!(res,(y+1,x))
        end
    end

    #Gauche
    if x>1 
        if grille[y-1,x]!= typemax(Int)
            push!(res,(y-1,x))
        end
    end

    #Haut
    if x>1 
        if grille[x,y]!= typemax(Int)
            push!(res,(x,y-1))
        end
    end

    #Bas
    if x<m
        if grille[y,x]!= typemax(Int)
            push!(res,(y,x+1))
        end
    end

    return res
end


#***********ALGO BFS*********************************
function algoBFS(fname::String, D::Tuple{Int,Int}, A::Tuple{Int,Int})
    map = loadMap(fname)
    n,m = size(map)
    visted = falses(n,m)
    #Initialisation de la file et ajout de D

    q = Queue{Tuple{Int,Int}}()
    enqueue!(q,D)
    b = true # Ce booléen permet de savoir si on a atteint A

    while b & !(isempty(q))
         u = dequeue!(q)
         if u==A 
            b = false
            continue
         end
         yp,xp = u
         visted[yp,xp] = true

         ngbr = neighbors(map,u)
         for s in ngbr
            enqueue!(q,s)
         end
    end

    return true,b
end