function [se2] = vec2se2(vec)
rho = vec(1:2);
theta = vec(3);

se2 = zeros(3, 3);
se2(1:2, 3) = rho;
se2(1:2, 1:2) = skew(theta);
end

function [vec] = se2vec(se2)
vec = zeros(3, 1);
vec(1:2) = se2(1:2, 3);
vec(3) = se2(2, 1);
end

