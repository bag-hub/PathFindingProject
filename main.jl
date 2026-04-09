##################################################
# main.jl
##################################################

# 1. Activation de l'environnement
using Pkg
Pkg.activate(".") 

# 2. Importation des librairies globales
using DataStructures
using Random
using Plots
gr()

# 3. Inclusion des fichiers sources
include("src/AMR.jl")
include("src/grid.jl")
include("src/astar2.jl") # A* SIPP (assurez-vous du bon nom de fichier)
include("src/crossDock.jl")

include("src/bfs.jl")
include("src/dijkstra.jl")
include("src/astar.jl")
include("src/glouton.jl")

println("--- Projet Path Finding chargé avec succès ! ---")
println("Vous pouvez maintenant tester les algorithmes.")

function algoMainP1(nomAlgo::String, fname::String, D::Tuple{Int64,Int64}, A::Tuple{Int64,Int64})
    if nomAlgo == "BFS"
        println("Lancement de BFS")
        time = @elapsed map,chemin,v,distance,nbVisited,cout = algoBFS(fname,D,A)

    elseif nomAlgo == "Astar"
        println("Lancement de A*")
        time = @elapsed map,chemin,v,distance,nbVisited,cout = algoAstar(fname,D,A)

    elseif nomAlgo == "Dijkstra"
        println("Lancement de Dijkstra")
        time = @elapsed map,chemin,v,distance,nbVisited,cout = algoDijkstra(fname,D,A)

    elseif nomAlgo == "Glouton"
        println("Lancement du Glouton")
        time = @elapsed map,chemin,v,distance,nbVisited,cout = algoGlouton(fname,D,A)
        
    else
        println("Erreur : Algorithme inconnu.")
        return nothing
    end
    
    plotVisited(map,chemin,v,distance,nbVisited,cout,D,A,nomAlgo) 
    return nothing
end


##################################################
# algoMainP2
#
# MAIN ENTRY POINT FOR PART 2
##################################################

function algoMainP2(scenario::String, n::Int=2)
    println("\n===================================")
    println(" MULTI-AGENT PATH PLANNING SYSTEM ")
    println(" Scénario : ", scenario)
    println("===================================")

    # Chemin vers la carte de l'entrepôt
    fname = "dat/cross_doc/cross_D14.map"
    amrs = Vector{AMR}()

    ##################################################
    # 1. INITIALISATION DES SCÉNARIOS
    ##################################################

    if scenario == "door_to_door"
        # Scénario standard : Déplacement de porte à porte
        # Teste la navigation de base et l'échelonnement des départs
        amrs = create_door_to_door_amrs(n)

    elseif scenario == "collision"
        # Scénario critique (Edge Conflict) : Face-à-face programmé
        # On force deux robots à emprunter exactement le même chemin en sens inverse
        # pour vérifier que le SIPP met l'un d'eux en attente.
        doors = get_doors()
        
        # Robot 1 : de la porte 1 vers la porte 8 (départ à t=0)
        push!(amrs, AMR(1, doors[1], doors[8], 0, 0, 0, Tuple{Int64,Int64}[]))
        
        # Robot 2 : de la porte 8 vers la porte 1 (départ à t=0)
        push!(amrs, AMR(2, doors[8], doors[1], 0, 0, 0, Tuple{Int64,Int64}[]))

    elseif scenario == "crossing"
        # Scénario d'intersection : Les trajectoires des robots se croisent
        # Permet d'observer la négociation dynamique aux carrefours
        amrs = create_crossing_amrs(n)

    elseif scenario == "multiple"
        # Scénario de charge (Stress-test) : Un trafic dense dans l'entrepôt
        # Réutilise la logique des croisements mais avec un grand nombre 'n' de robots
        # pour éprouver la limite de congestion et tester les "restarts".
        amrs = create_crossing_amrs(n)

    else
        # Gestion de sécurité en cas de paramètre incorrect
        println("Erreur : Scénario inconnu")
    end

    

    ##################################################
    # 2. AFFICHAGE DES ROBOTS
    ##################################################
    println("\nRobots initialisés :")
    for r in amrs
        println(" Robot ", r.id, " | Start: ", r.start, " -> Goal: ", r.goal, " | Start time: ", r.start_time)
    end

    ##################################################
    # 3. EXÉCUTION DE LA SIMULATION
    ##################################################
    println("\nLancement de la simulation SIPP...")
    result, env, global_finish = simulate_crossdock(fname, amrs)

    ##################################################
    # 4. AFFICHAGE DES RÉSULTATS ET STATISTIQUES
    ##################################################
    println("\n==============================")
    println(" RÉSULTATS")
    println("==============================")

    total_time = 0

    for r in result
        println("\nRobot ", r.id)
        println(" Start        : ", r.start)
        println(" Goal         : ", r.goal)
        println(" Start time   : ", r.start_time)
        println(" Finish time  : ", r.finish_time)
        println(" Path length  : ", length(r.path))

        total_time += r.finish_time
    end

    # Statistiques globales du système
    println("\n------------------------------")
    println("GLOBAL FINISH TIME  : ", global_finish)
    println("AVERAGE FINISH TIME : ", total_time / length(result))
    println("------------------------------")

    ##################################################
    # 5. GÉNÉRATION DE L'ANIMATION VISUELLE
    ##################################################
    animate_simulation(env, result, global_finish, "simulation_" * scenario * ".gif")

    return result
end

function main()
    println("\n===================================")
    println(" LAUNCHING CROSSDOCK SIMULATION")
    println("===================================\n")

    ##################################################
    # TEST 1 — Scénario Basique : 2 robots (Porte à Porte)
    ##################################################
    # Teste le comportement normal avec peu de congestion.
    #algoMainP2("door_to_door", 2)

    ##################################################
    # TEST 2 — Scénario Critique : Collision Frontale
    ##################################################
    # Force deux robots à se croiser dans un espace restreint (Edge Conflict).
    # Décommentez pour tester l'heuristique de Priorité + SIPP.
    #algoMainP2("collision")

    ##################################################
    # TEST 3 — Scénario Intermédiaire : 6 robots simultanés
    ##################################################
    # Permet d'observer comment les robots négocient les intersections.
    #algoMainP2("multiple", 6)

    ##################################################
    # TEST 4 — Stress Test (Charge Forte) : 14 robots
    ##################################################
    # Pousse l'algorithme dans ses retranchements pour tester les redémarrages (Restarts).
    #algoMainP2("multiple", 8)

    ##################################################
    # TEST RECOMMANDÉ
    ##################################################

    algoMainP2("crossing",6)

end

# Lancement automatique du script
main()