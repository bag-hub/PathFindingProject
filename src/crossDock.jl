##################################################
# crossDock.jl – Gestion des AMRs et réservation
##################################################

##################################################
# 1. STRUCTURES DE DONNÉES
##################################################

struct Mission
    start::Tuple{Int64,Int64}
    goal::Tuple{Int64,Int64}
    release_time::Int64
end

mutable struct AMR
    id::Int64
    start::Tuple{Int64,Int64}
    goal::Tuple{Int64,Int64}
    start_time::Int64
    duration::Int64
    finish_time::Int64
    path::Vector{Tuple{Int64,Int64}}
end

##################################################
# 2. GESTION DE L'ENVIRONNEMENT SPATIO-TEMPOREL
##################################################

function add_path_to_env!(env, path::Vector{Tuple{Int64,Int64}}, start_time::Int64)
    # Réservation de chaque case du chemin à l'instant t correspondant
    for i in 1:length(path)
        t = start_time + i - 1
        pos = path[i]

        if !haskey(env.dyn_obcls, pos)
            env.dyn_obcls[pos] = Vector{Tuple{Int64,Int64}}()
        end

        push!(env.dyn_obcls[pos], (t, t))
    end

    # ---------------------------------------------------------
    # LIBÉRATION DE LA CASE FINALE
    # ---------------------------------------------------------
    # Une fois que l'AMR a terminé sa mission, on s'assure que 
    # sa position finale ne bloque pas indéfiniment les autres robots.
    # On libère l'espace pour une fenêtre de temps donnée (ex: 20 unités).
    last_pos = path[end]
    finish_time = start_time + length(path)

    for t in finish_time+1 : finish_time + 20
        if haskey(env.dyn_obcls, last_pos)
            # Suppression de la réservation à l'instant t sur la case d'arrivée
            deleteat!(env.dyn_obcls[last_pos], findall(x -> x[1] == t, env.dyn_obcls[last_pos]))
        end
    end
end

##################################################
# 3. GÉNÉRATION DES SCÉNARIOS
##################################################

# Positions fixes des quais de chargement/déchargement
function get_doors()
    return [
        (1,4), (1,9), (1,14), (1,19), (1,24), (1,29), (1,34),
        (11,4), (11,9), (11,14), (11,19), (11,24), (11,29), (11,34)
    ]
end

# Scénario : Trafic croisé aléatoire (Stress-test des intersections)
function create_crossing_amrs(n::Int)
    doors = get_doors()
    amrs = Vector{AMR}()

    for i in 1:n
        start = doors[i]
        
        # Sélection d'une destination aléatoire différente du point de départ
        possible_goals = copy(doors)
        deleteat!(possible_goals, findfirst(x -> x == start, possible_goals))
        goal = rand(possible_goals)
        
        # Introduction d'une variabilité dans les départs (0 à 5)
        start_time = rand(0:5)
        duration = abs(start[1] - goal[1]) + abs(start[2] - goal[2])
        
        push!(amrs, AMR(i, start, goal, start_time, duration, 0, Tuple{Int64,Int64}[]))
    end

    return amrs
end

# Scénario : Déplacement structuré d'un quai à son opposé
function create_door_to_door_amrs(n::Int)
    doors = get_doors()
    amrs = Vector{AMR}()

    for i in 1:n
        start = doors[i]
        
        # Le modulo garantit un trajet traversant l'entrepôt
        goal_index = mod(i + 6, length(doors)) + 1
        goal = doors[goal_index]
        
        # Échelonnement strict des départs pour lisser la charge initiale
        start_time = i - 1
        duration = abs(start[1] - goal[1]) + abs(start[2] - goal[2])
        
        push!(amrs, AMR(i, start, goal, start_time, duration, 0, Tuple{Int64,Int64}[]))
    end

    return amrs
end

##################################################
# 4. PLANIFICATION ET HEURISTIQUES PRIORITAIRES
##################################################

# Heuristique : Priorité aux trajets les plus longs, avec mécanisme VIP
function estimated_priority(amr::AMR, priority_boost_id::Int64)
    # Si le robot a causé un échec au cycle précédent, il devient prioritaire absolu
    if amr.id == priority_boost_id
        return -999999
    end
    # Sinon, priorité basée sur la distance de Manhattan estimée (plus long = prioritaire)
    return -heuristic(amr.start, amr.goal)
end

function build_priority_queue(amrs::Vector{AMR}, priority_boost_id::Int64)
    pq = PriorityQueue{AMR, Int64}()
    for robot in amrs
        pq[robot] = estimated_priority(robot, priority_boost_id)
    end
    return pq
end

# Planification d'un AMR unique (Replanification immédiate en cas d'échec)
function plan_single_amr!(env::Environment, robot::AMR)
    max_attempts = 5

    for attempt in 1:max_attempts
        println("   Tentative ", attempt, " pour AMR ", robot.id)

        # Appel de l'algorithme A* SIPP
        result = algoAstar2(env, robot.start, robot.goal, robot.start_time)

        if result !== nothing
            map, path, v, d, n, makespan = result
            
            robot.path = path
            robot.finish_time = robot.start_time + makespan
            
            # Validation globale de la trajectoire
            add_path_to_env!(env, path, robot.start_time)
            
            println("   Chemin trouvé pour AMR ", robot.id)
            return true
        end

        # Comportement NO WAIT : recalcul immédiat
        println("   Aucun chemin pour AMR ", robot.id, " → recalcul immédiat")
    end

    println("   Échec après ", max_attempts, " tentatives pour AMR ", robot.id)
    return false
end

##################################################
# 5. ORCHESTRATEUR PRINCIPAL (CROSS-DOCKING)
##################################################

function simulate_crossdock(fname::String, amrs::Vector{AMR})
    max_restarts = 5
    restarts = 0
    priority_boost_id = -1
    
    global_finish = 0
    env = nothing

    # Boucle de redémarrage : Failed-First Restart
    while restarts <= max_restarts
        println("\n--- TENTATIVE DE PLANIFICATION (Essai ", restarts + 1, ") ---")
        
        env = emptyEnv(fname)
        global_finish = 0
        success_all = true

        # Réinitialisation des chemins de la tentative précédente
        for r in amrs
            r.path = Tuple{Int64,Int64}[]
        end

        # Construction de la file d'attente avec prise en compte du VIP éventuel
        pq = build_priority_queue(amrs, priority_boost_id)

        # Planification séquentielle
        while !isempty(pq)
            robot = dequeue!(pq)
            println("Planning AMR ", robot.id)
            
            success = plan_single_amr!(env, robot)

            if success
                # Mise à jour du Makespan global
                global_finish = max(global_finish, robot.finish_time)
            else
                println(" IMPOSSIBLE DE PLANIFIER L'AMR ", robot.id)
                println(" Déclenchement du Failed-First Restart...")
                
                # Le robot bloquant reçoit le boost de priorité pour la prochaine boucle
                priority_boost_id = robot.id
                success_all = false
                break
            end
        end

        # Si tous les robots ont trouvé un chemin valide, on arrête la boucle
        if success_all
            println("\n SUCCÈS TOTAL DE LA PLANIFICATION ! Temps de fin global : ", global_finish)
            return amrs, env, global_finish
        end

        restarts += 1
    end

    println("\n ÉCHEC TOTAL après ", max_restarts, " redémarrages. Problème insolvable avec cette heuristique.")
    return amrs, env, global_finish
end