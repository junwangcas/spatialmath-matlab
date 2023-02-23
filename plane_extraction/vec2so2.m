function [so2] = vec2so2(vec)
theta = vec(1);

so2 = zeros(2, 2);
so2(1:2, 1:2) = skew(theta);
end



