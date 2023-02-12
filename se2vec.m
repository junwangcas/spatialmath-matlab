function [vec] = se2vec(se2)
vec = zeros(3, 1);
vec(1:2) = se2(1:2, 3);
vec(3) = se2(2, 1);
end

