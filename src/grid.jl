#**************************************************

# Struct Environement ajouté dans la 2e partie du projet pour ré-adapter 
struct Environment
    # La carte 
    map::Matrix{Int64}
    # Le dictionnaire des intervalles où les positions 
    #(y,x) => tableau de paires d'intervalles
    dyn_obcls::Dict{Tuple{Int64,Int64},Vector{Tuple{Int64,Int64}}}
end 

# Cette fonction permet de créer un environnement sans contrainte pour l'initialisation au début des missions des AMR
function emptyEnv(fname::String)
    map = loadMap2(fname)
    d = Dict{Tuple{Int64,Int64}, Vector{Tuple{Int64,Int64}}}()
    return Environment(map, d)
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

#Fonction pour charger une carte et créer la matrice correspondant à  cette carte pour la partie 2
function loadMap2(fname::String)
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
                carte[i,j] = 2
            end
        end
    end
    return carte
end

function neighbors(env::Environment, s::Tuple{Int64,Int64}, t::Int64)
    # Extraction des coordonnées actuelles de l'état
    y, x = s
    grille = env.map
    n, m = size(grille)

    # Initialisation de la liste chaînée stockant les voisins valides
    res = MutableLinkedList{Tuple{Int64,Int64}}()

    # Définition des mouvements possibles : Droite, Gauche, Haut, Bas, et Attente sur place
    moves = [(0, 1), (0, -1), (-1, 0), (1, 0), (0, 0)]

    for (dy, dx) in moves
        ny = y + dy
        nx = x + dx

        # 1. Vérification des limites de la carte (Bounds checking)
        if 1 <= ny <= n && 1 <= nx <= m
            
            # 2. Vérification des obstacles statiques 
            # typemax(Int64) représente un mur ou une zone infranchissable
            if grille[ny, nx] != typemax(Int64)
                
                # Le coût du mouvement dépend du type de la case (ex: déplacement normal vs zone lente)
                move_cost = grille[ny, nx]
                
                # Calcul de l'heure d'arrivée prévue sur la case adjacente
                arrival = t + move_cost

                # 3. Vérification des obstacles dynamiques (Dimension spatio-temporelle)
                # On s'assure qu'aucun autre robot ne se trouve sur cette case à l'instant d'arrivée
                if !is_collision(env, ny, nx, arrival)
                    push!(res, (ny, nx))
                end
            end
        end
    end

    return res
end

function heuristic(A::Tuple{Int64,Int64}, B::Tuple{Int64,Int64})
    return abs(A[1] - B[1]) + abs(A[2] - B[2])
end

# Visualisation des algos de recherche de chemins 
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

#***********Partie 2 du projet**********

# Fonction pour vérifier si la case (y, x) est occupée à l'instant t
function is_collision(env::Environment, y::Int64, x::Int64, t::Int64)
    # Si la case n'a aucune réservation, c'est libre
    if !haskey(env.dyn_obcls, (y, x))
        return false
    end
    
    # Sinon, on vérifie si t tombe dans un des intervalles réservés
    for (start_t, end_t) in env.dyn_obcls[(y, x)]
        if start_t <= t <= end_t
            return true # Collision !
        end
    end
    
    return false
end

##################################################
# trouver le prochain intervalle libre
##################################################

function next_free_time(env::Environment, pos::Tuple{Int64,Int64}, t::Int64)

    if !haskey(env.dyn_obcls, pos)
        return t
    end

    for (start_t,end_t) in env.dyn_obcls[pos]

        if start_t <= t <= end_t
            return end_t + 1
        end
    end

    return t
end

##################################################
# ANIMATION SIPP 
##################################################

function animate_simulation(env::Environment, amrs::Vector{AMR}, global_finish::Int, fname::String="simulation.gif")
    println("Génération de l'animation en cours (cela peut prendre quelques secondes)...")
    
    n, m = size(env.map)
    
    # Création de la carte de fond (0 = vide, 1 = obstacle)
    base_map = zeros(Int, n, m)
    for y in 1:n
        for x in 1:m
            if env.map[y,x] == typemax(Int64) || env.map[y,x] == 8
                base_map[y,x] = 1
            end
        end
    end

    # Couleurs pour différencier les robots
    colors = [:red, :blue, :green, :orange, :purple, :cyan]

    # Astuce : On ajoute 5 frames supplémentaires pour créer une pause visuelle à la fin du GIF
    pause_frames = 5 

    anim = @animate for t in 0:(global_finish + pause_frames)
        # 1. Dessiner l'entrepôt
        p = heatmap(
            base_map,
            yflip=true,
            aspect_ratio=1,
            legend=false,
            axis=false,
            colorbar=false,
            c = [:white, :gray], 
            title="Cross-Docking SIPP | Temps t = $t"
        )

        # 2. Dessiner chaque robot et son historique
        for (idx, robot) in enumerate(amrs)
            # Assigner une couleur unique au robot
            color = colors[mod1(idx, length(colors))]
            
            # Afficher la position de DÉPART (étoile) et d'ARRIVÉE (croix)
            scatter!(p, [robot.start[2]], [robot.start[1]], markershape=:star, color=color, markersize=6)
            scatter!(p, [robot.goal[2]], [robot.goal[1]], markershape=:xcross, color=color, markersize=6)

            if t >= robot.start_time
                # Index actuel dans le chemin du robot
                path_idx = t - robot.start_time + 1
                
                # --- NOUVEAUTÉ 1 : Tracer l'historique du déplacement ---
                # On détermine jusqu'où le robot a voyagé (max_idx empêche de déborder du tableau)
                max_idx = min(path_idx, length(robot.path))
                
                if max_idx > 0
                    # On extrait toutes les coordonnées parcourues jusqu'à présent
                    path_history = robot.path[1:max_idx]
                    xs_history = [pos[2] for pos in path_history]
                    ys_history = [pos[1] for pos in path_history]
                    
                    # On trace une ligne semi-transparente (alpha=0.5) pour marquer la trace du robot
                    plot!(p, xs_history, ys_history, color=color, linewidth=3, alpha=0.5)
                end
                
                # --- NOUVEAUTÉ 2 : Position actuelle ---
                current_pos = robot.start
                if path_idx <= length(robot.path) && path_idx > 0
                    current_pos = robot.path[path_idx]
                elseif path_idx > length(robot.path)
                    current_pos = robot.goal
                end
                
                # Dessiner le robot (un cercle plein) sur sa position exacte à l'instant t
                scatter!(p, [current_pos[2]], [current_pos[1]], markershape=:circle, color=color, markersize=8)
            end
        end
    end
    
    println("Sauvegarde de l'animation dans : ", fname)
    
    # NOUVEAUTÉ 3 : fps = 1 au lieu de 2 pour ralentir l'animation (1 image = 1 seconde)
    gif(anim, fname, fps = 1) 
    
    println("Animation terminée avec succès !")
end