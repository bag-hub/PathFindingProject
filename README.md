# PathFindingProject

Compilation et exécution du projet partie 1

Il faut se rendre dans le repertoire où se trouve le dossier du PathFindingProject, accéder à ce dossier et compiler avec le REPL : 
include("src/PathFindingProject.jl")
map = algoDijkstra("src/fname.map",D,A)
map = algoAstar("src/fname.map",D,A)
map = algoGlouton("src/fname.map",D,A)



📘 Projet : Recherche de Chemins et Planification Multi-Agents (Cross-Docking)
Ce projet implémente un système de navigation pour des Robots Mobiles Autonomes (AMR) dans un environnement logistique contraint de type cross-docking.

Il valide d'abord des approches de recherche de chemin statiques (Partie 1), avant de déployer une architecture multi-agents robuste basée sur une heuristique spatio-temporelle (Partie 2).

🚀 1. Prérequis et Installation
Le projet est développé en Julia. Pour garantir son exécution, les bibliothèques suivantes sont requises :

DataStructures.jl (Pour les files de priorité temporelles)

Plots.jl (Pour la génération des Heatmaps et des animations GIF)

Note : Le code inclut une commande Pkg.activate(".") au lancement pour charger automatiquement l'environnement local si vous avez fourni les fichiers Project.toml et Manifest.toml.

🧠 2. Architecture de la Solution (Partie 2)
Pour résoudre le problème de navigation multi-agents sans collision, l'algorithme repose sur trois piliers :

A Spatio-Temporel (Pseudo-SIPP)* : L'espace de recherche est étendu à trois dimensions (y, x, t). L'algorithme calcule le prochain intervalle de temps libre sur une case pour éviter les collisions avec les robots déjà planifiés.

Prioritized Planning : Les robots sont planifiés séquentiellement. Les robots ayant les trajets théoriques les plus longs sont planifiés en priorité (Heuristique de priorité).

Failed-First Restart : Si un robot est bloqué par ceux déjà planifiés, la simulation s'annule, le robot bloqué reçoit une priorité absolue (VIP), et le système relance le calcul complet. Le système autorise jusqu'à 5 redémarrages.

🎯 3. Comment tester les Algorithmes ?
Tout le code d'exécution est centralisé à la fin du script principal, dans la fonction main(). Pour évaluer la robustesse du système, 4 scénarios de test ont été pré-configurés.

Pour tester un scénario, il vous suffit de décommenter la ligne correspondante dans la fonction main() et d'exécuter le script :

Julia
function main()
    # ---------------------------------------------------------
    # TEST 1 : Scénario Basique (2 robots, de porte à porte)
    # Permet de vérifier le fonctionnement nominal de l'algorithme SIPP.
    # ---------------------------------------------------------
    algoMainP2("door_to_door", 2)

    # ---------------------------------------------------------
    # TEST 2 : Scénario Critique (Collision Frontale - Face-à-face)
    # Force deux robots à se croiser dans un couloir étroit.
    # Démontre la capacité de l'algorithme à faire patienter un robot.
    # ---------------------------------------------------------
    # algoMainP2("collision")

    # ---------------------------------------------------------
    # TEST 3 : Scénario Intermédiaire (6 robots simultanés)
    # Observe la négociation des croisements multiples.
    # ---------------------------------------------------------
    # algoMainP2("crossing", 6)

    # ---------------------------------------------------------
    # TEST 4 : Stress Test (14 robots simultanés)
    # Pousse l'algorithme dans ses retranchements pour déclencher
    # la logique de "Failed-First Restart".
    # ---------------------------------------------------------
    # algoMainP2("multiple", 14)
end
📊 4. Sorties Attendues (Outputs)
Lors de l'exécution d'un scénario, le programme générera deux types de résultats :

Logs dans la Console : Vous pourrez suivre en temps réel les tentatives de planification, les éventuels "Restarts" en cas d'échec, ainsi que les statistiques finales (Temps d'arrivée individuel, Temps global/Makespan).

Animation Visuelle (GIF) : À la fin de chaque exécution réussie, un fichier de type simulation_nom_du_scenario.gif sera généré et sauvegardé dans le dossier racine. Ce fichier permet de visualiser pas à pas les déplacements et l'évitement des collisions (marqué par une trace de couleur pour chaque robot).
