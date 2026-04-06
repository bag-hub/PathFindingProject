#**************************************************

# Struct Environement ajouté dans la 2e partie du projet pour ré-adapter 
struct Environnement
    # La carte 
    map::Matrix{Int64}
    # Le dictionnaire des intervalles où les positions 
    dyn_obcls::Dict{Tuple{Int64,Int64},Vector{Tuple{Int64,Int64}}}
end 

# Cette permet de créer une carte map simple utilisée dans la première partie du projet
function empty_env(map::Matrix{Int64})
    return Environnement(map,Vector{Tuple{Int64,Int64}}())
end


#Fonction pour charger une carte et créer la matrice correspondant à  cette carte
function loadMap(fname::String)
    #Lecture fichier
    lines = open(fname,"r") do f
        readlines(f)
    end

    #Les 4 premières lignes représentent les métadonnées
    n = length(lines)-4 #Nombres de lignes 
    m = length(lines[5]) #Nombres de colonnes
    carte = Matrix{Int64}(undef,n,m)
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
                carte[i,j] = typemax(Int64)
            end
        end
    end
    return carte
end

function neighbors(grille::Matrix{Int64},s::Tuple{Int64,Int64})
    y,x = s
    n,m = size(grille)
    res = MutableLinkedList{Tuple{Int64,Int64}}()
    
    # Droite
    if x<m
        if grille[y,x+1]!= typemax(Int64)
            push!(res,(y,x+1))
        end
    end
    
    # Gauche
    if x>1 
        if grille[y,x-1]!= typemax(Int64)
            push!(res,(y,x-1))
        end
    end

    # Haut
    if y>1 
        if grille[y-1,x]!= typemax(Int64)
            push!(res,(y-1,x))
        end
    end

    # Bas
    if y<n
        if grille[y+1,x]!= typemax(Int64)
            push!(res,(y+1,x))
        end
    end

    return res
end


function plotVisited(map, chemin, visited, distance, nbVisited, cout, D, A,nameAlgo)
    n, m = size(map)

    # 0 = non visité, 1 = visité
    displayMap = zeros(Int, n, m)
    for (y,x) in visited
        displayMap[y,x] = 1
    end

    # Heatmap des points visités
    p = heatmap(
        displayMap,
        yflip=true,
        aspect_ratio=1,
        legend=:outertopright,
        #label="Point visité",
        axis=false,
        colorbar=false,
        c = [:white, :blue],   # 0 = blanc, 1 = bleu
        title="Exploration et Chemin Final pour "*nameAlgo)
    # Le point(Nan,NaN) "fantôme" pour forcer l'entrée dans la légende car "colorbar=false" force la disparition du label
    scatter!(
        p, [NaN], [NaN], 
        color=:blue, 
        markershape=:square, # Un carré rappelle la forme des cases de la grille
        label="Point visité"
    )
    #
    # On trace le chemin plus court trouvé
    # On vérifie que le chemin n'est pas vide pour éviter une erreur
    if !isempty(chemin)
        # On extrait X (indice 2) et Y (indice 1) des points du chemin
        xs = [pos[2] for pos in chemin]
        ys = [pos[1] for pos in chemin]
        
        # On trace la ligne par-dessus la heatmap
        plot!(p, xs, ys, color=:yellow, linewidth=2, label="Chemin trouvé")
    end

    # Marquer D et A
    scatter!(
        [D[2]], [D[1]],
        markersize=4, marker=:circle, color=:red, label="Départ (D)")
    scatter!(
        [A[2]], [A[1]],
        markersize=4, marker=:circle, color=:green, label="Arrivé (A)")

    # Ajouter texte à l'intérieur du plot
    info_text = "
    Distance D->A: $distance, 
    Number of states evaluated: $nbVisited, Coût réelle D->A: $cout "
    annotate!(1, n, text(info_text, :black, 7, :left))

    #Sauvegarde du plot
    savefig(p, "output"*nameAlgo*".png")

    display(p)  # Affichage
end