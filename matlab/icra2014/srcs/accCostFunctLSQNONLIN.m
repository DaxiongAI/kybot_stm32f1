%% accCostFunctLSQNONLIN.m

function [ res_vector ] = accCostFunctLSQNONLIN( E, a_hat )

misalignmentMatrix = [1, -E(1), E(2); 0, 1, -E(3); 0, 0, 1];
scalingMtrix = diag([E(4), E(5), E(6)]);

a_bar = misalignmentMatrix*scalingMtrix*(a_hat) - (diag([E(7), E(8), E(9)])*ones(3, length(a_hat)));

% Magnitude taken from tables 
%magnitude = 9.81744;
%武汉的重力加速度
magnitude = 9.79361;

residuals = zeros(length(a_bar(1,:)), 1);

for i = 1:length(a_bar(1,:))
    residuals(i,1) = magnitude^2 - (a_bar(1,i)^2 + a_bar(2,i)^2 + a_bar(3,i)^2);
end

res_vector = residuals;

end
