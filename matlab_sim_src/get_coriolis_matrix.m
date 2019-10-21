function C = get_coriolis_matrix(M,q,qdot)
% get_coriolis
% Computes coriolis matrix from mass matrix
    C = sym('C', size(M));
    for i = 1:size(M,1)
        for j = 1:size(M,2)
            C(i,j) = 0;
            for k = 1:size(M,2)
                C(i,j) = C(i,j) + 0.5*(diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(k,j), q(i))) * qdot(k);
            end
        end
    end
    C = simplify(C);
end

