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

#Cette fonction nous permet d'obtenir une matrice de booléen pour connaitre les cases déjà visitées.

#=function chargerCarteVisite(fname::String)
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
            if car[j]=='@'
                carte[i,j] = 0
            else
                carte[i,j] = 1
            end
        end
    end
    return carte
    print(n,m)
end
=#

function neighbors(grille::Matrix{Int},x::Int,y::Int)
    n,m = size(grille)
    directions = [(0,1),(0,-1),(-1,0),(1,0)]
    res = Tuple{Int,Int}[]
    for d in directions
        dx,dy = d
        if grille[x+dx,y+dy]!= typemax(Int)
            push!(res,(x+dx,y+dy))
        end
    end
    return res
end
