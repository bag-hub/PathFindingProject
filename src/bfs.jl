
using DataStructures

function algoBFS(fname::String, D::Tuple{Int,Int}, A::Tuple{Int,Int})
    map = loadMap(fname)
    n,m = size(map)
    visted = falses(n,m)
    #Initialisation de la file et ajout de D
    q = Queue{Int}()
    enqueue!(q,D)
    b = true

    while b & !(isempty(q))
         xp,xp = dequeue!(q)
         visted[xp,yp] = false
    

    return map
end