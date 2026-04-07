# main.jl

# 1. Activation de l'environnement (Très bonne pratique grâce à ton Project.toml)
using Pkg
Pkg.activate(".") 

# 2. Importation des librairies globales
using DataStructures
using Plots
gr() # Pour sélectionner le moteur graphique utilisé par Plots pour afficher les graphiques

# 3. Inclusion des fichiers sources (l'ordre compte !)
# D'abord les dépendances "basses"
include("src/grid.jl")
include("src/utils.jl")
include("src/AMR.jl")
include("src/crossDock.jl")
# Ensuite les algorithmes
include("src/bfs.jl")
include("src/dijkstra.jl")
include("src/astar.jl")
include("src/glouton.jl")

println("--- Projet Path Finding chargé avec succès ! ---")
println("Vous pouvez maintenant tester les algorithmes. Exemple :")

function algoMainP1(nomAlgo::String, fname::String, D::Tuple{Int64,Int64}, A::Tuple{Int64,Int64})
    if nomAlgo == "BFS"
        println("Lancement de BFS")

        # map est retourné par chaque algo pour éviter le coût de la complexité de retraiter la carte

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
    
    println("Time (s)                :",time)

    plotVisited(map,chemin,v,distance,nbVisited,cout,D,A,nomAlgo) 
    return nothing
end

# Dans cette partie 2 du projet , on doit passer en paramètre le nombre de robots 
#que l'on veut pour une simulation de l'algo et la taille du CrossDocking
function algoMainP2(n::UInt64,len::UInt16)
    return nothing
end

function algoMain()
    return nothing
end 