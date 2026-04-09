#Cross Dock et gestion de la partie 2 du projet

##################################################
# crossDock.jl
##################################################

struct Mission
    start::Tuple{Int64,Int64}
    goal::Tuple{Int64,Int64}
    release_time::Int64
end

##################################################
# RESERVATION TABLE
##################################################

function add_path_to_env!(env, path, start_time)
    ##################################################
    # LOGIC:
    # Reserve each position at its time
    ##################################################
    
    for i in 1:length(path)
        t = start_time + i - 1
        pos = path[i]

        if !haskey(env.dyn_obcls, pos)
            env.dyn_obcls[pos] = Vector{Tuple{Int64,Int64}}()
        end

        push!(env.dyn_obcls[pos], (t, t))
    end
end

##################################################
# PLAN SINGLE ROBOT
##################################################

function plan_single_amr!(env, robot::AMR)
    # Paramètres de flexibilité temporelle
    max_attempts = 3
    delay = 2 # Décalage du temps de départ en cas d'échec (en unités de temps / minutes)

    # Boucle d'essais : on tente de planifier le trajet plusieurs fois en retardant le départ si nécessaire
    for attempt in 1:max_attempts
        
        # Appel de l'algorithme A* spatio-temporel (SIPP) en prenant en compte l'heure de départ
        result = algoAstar2(env, robot.start, robot.goal, robot.start_time)

        # Si un chemin valide a été trouvé (pas de conflit absolu)
        if result !== nothing
            map, path, v, d, n, c = result

            # Sauvegarde de la trajectoire trouvée dans l'entité du robot
            robot.path = path
            
            # Calcul du temps de fin de mission (start_time + makespan retourné par A*)
            robot.finish_time = robot.start_time + c

            # Mise à jour de la table de réservation globale de l'environnement
            # Cela garantit que les prochains robots contourneront ce chemin
            add_path_to_env!(env, path, robot.start_time)

            return true # Planification réussie
        end

        # En cas d'échec (ex: embouteillage insurmontable à t=start_time), 
        # on décale le départ du robot dans le temps avant de réessayer.
        println("   Aucun chemin pour AMR ", robot.id, " -> nouvelle tentative dans ", delay, " minutes")
        robot.start_time += delay
    end

    # Si le robot ne trouve toujours pas de chemin après max_attempts, on renvoie un échec global
    return false
end

##################################################
# PRIORITY HEURISTIC (Longest Path First + VIP)
##################################################

function estimated_priority(amr::AMR, priority_boost_id::Int64)
    ##################################################
    # LOGIC:
    # 1. If this robot caused a failure last time,
    #    give it absolute priority (-999999).
    # 2. Else, prioritize longest distance first.
    ##################################################
    
    if amr.id == priority_boost_id
        return -999999
    end
    
    # On suppose ici que ta fonction heuristic(A, B) est dispo (distance de Manhattan)
    # Plus le trajet théorique est long, plus le chiffre est négatif (donc sort en 1er)
    return -heuristic(amr.start, amr.goal)
end

##################################################
# BUILD PRIORITY QUEUE
##################################################

function build_priority_queue(amrs::Vector{AMR}, priority_boost_id::Int64)
    ##################################################
    # LOGIC:
    # Insert all robots into the queue
    # ordered by our custom priority heuristic
    ##################################################
    
    pq = PriorityQueue{AMR, Int64}()

    for robot in amrs
        pq[robot] = estimated_priority(robot, priority_boost_id)
    end

    return pq
end

##################################################
# DOOR POSITIONS (14 quais)
##################################################

function get_doors()

    return [(1,4),(1,9),(1,14),(1,19),(1,24),(1,29),(1,34),
        (11,4),(11,9),(11,14),(11,19),(11,24),(11,29),(11,34)]

end

##################################################
# CREATE AMRS FROM DOOR TO DOOR
##################################################

function create_door_to_door_amrs(n::Int)
    # Récupération de la liste des coordonnées des portes (quais de l'entrepôt)
    doors = get_doors()
    amrs = Vector{AMR}()

    # Boucle de création de 'n' robots
    for i in 1:n
        # Le point de départ est assigné à la porte 'i'
        start = doors[i]

        # Le point d'arrivée est décalé pour forcer le robot à traverser l'entrepôt.
        # Le modulo permet de reboucler au début du tableau si l'index dépasse le nombre total de portes.
        goal_index = mod(i + 6, length(doors)) + 1
        goal = doors[goal_index]

        # Départs différés : chaque robot part 1 minute (ou unité de temps) après le précédent 
        # Cela permet d'éviter un engorgement immédiat au temps t=0
        start_time = i - 1

        # Calcul théorique de la durée du trajet sans obstacle (Distance de Manhattan)
        duration = abs(start[1] - goal[1]) + abs(start[2] - goal[2])

        # Initialisation du robot avec son identifiant, ses coordonnées, ses temps, et un chemin vide
        push!(
            amrs, 
            AMR(i, start, goal, start_time, duration, 0, Tuple{Int64,Int64}[])
        )
    end

    return amrs
end

##################################################
# MAIN SIMULATION (With Failed-First Restart)
##################################################

function simulate_crossdock(fname::String, amrs::Vector{AMR})
    
    max_restarts = 5
    restarts = 0
    priority_boost_id = -1
    
    global_finish = 0
    env = nothing # Déclaré ici pour pouvoir le retourner à la fin

    ##################################################
    # MAIN RESTART LOOP
    ##################################################
    while restarts <= max_restarts
        println("\n--- TENTATIVE DE PLANIFICATION (Essai ", restarts + 1, ") ---")
        
        ##################################################
        # RESET ENVIRONMENT & VARIABLES
        ##################################################
        env = emptyEnv(fname)
        global_finish = 0
        success_all = true

        # Vider les chemins précédents des robots en cas de restart
        for r in amrs
            r.path = Tuple{Int64,Int64}[]
        end

        ##################################################
        # BUILD PRIORITY QUEUE
        ##################################################
        pq = build_priority_queue(amrs, priority_boost_id)

        ##################################################
        # PLAN ROBOTS ONE BY ONE
        ##################################################
        while !isempty(pq)
            robot = dequeue!(pq)
            println("Planning AMR ", robot.id)
            
            success = plan_single_amr!(env, robot)

            if success
                ##################################################
                # UPDATE GLOBAL FINISH
                ##################################################
                if robot.finish_time > global_finish
                    global_finish = robot.finish_time
                end
            else
                ##################################################
                # PLANNING FAILED -> TRIGGER RESTART
                ##################################################
                println(" IMPOSSIBLE DE PLANIFIER L'AMR ", robot.id)
                println(" Déclenchement du Failed-First Restart...")
                
                # Ce robot devient le VIP pour la prochaine tentative
                priority_boost_id = robot.id
                success_all = false
                break # On casse la boucle while interne pour recommencer depuis le début
            end
        end

        ##################################################
        # CHECK SUCCESS
        ##################################################
        if success_all
            println("\n SUCCÈS TOTAL DE LA PLANIFICATION ! Temps de fin global : ", global_finish)
            return amrs, env, global_finish
        end

        restarts += 1
    end

    ##################################################
    # IF WE EXCEEDED MAX RESTARTS
    ##################################################
    println("\n ÉCHEC TOTAL après ", max_restarts, " redémarrages. Problème insolvable avec cette heuristique.")
    return amrs, env, global_finish
end