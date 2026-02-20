mutable struct Graphe
    adj::Vector{Vector{Tuple{Int,Int}} }   #liste d'adjacence
    nbSommets::Int                         #nombre de sommets
    #On aura besoin de nbLignes::Int, nbCol pour reconstruire le chemin
    nbLignes::Int
    nbCol::Int
end

function graphe(fname::String)::Graphe
    #Lecture fichier
    lines = open(fname,"r") do f
        readlines(f)
    end

    #Les 4 premières lignes représentent les métadonnées
    n = length(lines)-4  #Nombres de lignes 
    m = length(lines[5]) #Nombres de colonnes
    #On met end et pas n car n=n = length(lines)-4 
    tab_lignes = [collect(strip(l)) for l in lines[5:end]]

    nbSommets = n*m
    adj = [Vector{Tuple{Int,Int}}() for _ in 1:nbSommets]
    #directions = [(0,1)(1,0),(0,-1),(-1,0)]
    for i in 1:n
        for j in 1:m
            id = (i-1)*m+j

            #directions : Gauche,Droite, Haut,Bas
            #Gauche
            if j-1>=1
                c = tab_lignes[i][j-1]
                if c=='@'
                    
                elseif c=='.'
                    push!(adj[id], (id-1,1))
                elseif c=='S'
                    push!(adj[id],(id-1,5))
                else 
                    push!(adj[id],(id-1,8))
                end
            end

            #Droite
            if j+1<=m
                c = tab_lignes[i][j+1]
                if c=='@'
                
                elseif c=='.'
                    push!(adj[id],(id+1,1))
                elseif c=='S'
                    push!(adj[id],(id+1,5))
                else 
                    push!(adj[id],(id+1,8))
                end
            end

            #Haut
            if i-1>=1
                c = tab_lignes[i-1][j]
                if c=='@'
            
                elseif c=='.'
                    push!(adj[id],(id-m,1))
                elseif c=='S'
                    push!(adj[id],(id-m,5))
                else 
                    push!(adj[id],(id-m,8))
                end
            end

            #Bas
            if i+1<=n
                c = tab_lignes[i+1][j]
                if c=='@'
            
                elseif c=='.'
                    push!(adj[id],(id+m,1))
                elseif c=='S'
                    push!(adj[id],(id+m,5))
                else 
                    push!(adj[id],(id+m,8))
                end
            end

        end
    end
    
    return Graphe(adj,nbSommets,n,m)
end
