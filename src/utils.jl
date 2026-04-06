function heuristic(A::Tuple{Int64,Int64}, B::Tuple{Int64,Int64})
    return abs(A[1] - B[1]) + abs(A[2] - B[2])
end