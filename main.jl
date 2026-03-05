#=
include("src/PathFindingProject.jl")
algoBFS("dat/arena.map",(2,5),(4,17))
map = algoDijkstra("dat/arena.map",(2,5),(4,17))
map = algoAstar("dat/arena.map",(2,5),(4,17))
map = algoGlouton("dat/arena.map",(2,5),(4,17))
=#