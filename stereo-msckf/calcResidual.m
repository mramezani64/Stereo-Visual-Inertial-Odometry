function [r_j] = calcResidual(p_f_G,p2_f_G, camStates_L, camStates_R, measurements_L, measurements_R)
%CALCRESIDUAL Calculates the residual for a feature position

% measurements is 2 x M_j
% camStates is a cell array of the camState structs for the states
%   included in measurements
    r_j = NaN(4*size(camStates_L,2), 1);
    for i = 1:size(camStates_L,2)
        
        C_CG = quatToRotMat(camStates_L{i}.q_CG);
        p_f_C = C_CG * (p_f_G - camStates_L{i}.p_C_G);
        
        C2_CG = quatToRotMat(camStates_R{i}.q2_CG);
        p2_f_C = C2_CG * (p2_f_G - camStates_R{i}.p2_C_G);
        
        zhat_i_j = p_f_C(1:2)/p_f_C(3);
        
        zhat2_i_j = p2_f_C(1:2)/p2_f_C(3); % For the second cam
        
%         iStart = 2*(i-1)+1;
%         iEnd = 2*i;
        r_j(4*i-3:4*i-2) = measurements_L(:,i) - zhat_i_j;
        r_j(4*i-1:4*i) = measurements_R(:,i) - zhat2_i_j;
    end
        
end