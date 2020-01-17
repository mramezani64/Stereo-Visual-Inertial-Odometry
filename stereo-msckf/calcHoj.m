function [H_o_j, A_j, H_x_j] = calcHoj(p_f_G, p2_f_G, msckfState, camStateIndices)
%CALCHOJ Calculates H_o_j according to Mourikis 2007
% Inputs: p_f_G: feature location in the Global frame
%         msckfState: the current window of states
%         camStateIndex: i, with camState being the ith camera pose in the window       
% Outputs: H_o_j, A


N = length(msckfState.camStates_L);
M = length(camStateIndices);
H_x_j = zeros(4*M,12 + 2*6*N);
H1_f_j = zeros(2*M, 3);
H2_f_j = zeros(2*M, 3);
H1_x_j = zeros(2*M, 12 + 6*N); % for 2 cams we have 4 rows and 12+6*2*N columns
H2_x_j = zeros(2*M, 6*N);

c_i = 1;
for camStateIndex = camStateIndices
    camState_L = msckfState.camStates_L{camStateIndex};
    camState_R = msckfState.camStates_R{camStateIndex}; % For the second cam

    C_CG = quatToRotMat(camState_L.q_CG);
    C2_CG = quatToRotMat(camState_R.q2_CG);
    %The feature position in the camera frame
    p_f_C = C_CG*(p_f_G - camState_L.p_C_G);
    p2_f_C = C2_CG*(p2_f_G - camState_R.p2_C_G);

    X_L = p_f_C(1);
    Y_L = p_f_C(2);
    Z_L = p_f_C(3);
    % For the second cam
    X_R = p2_f_C(1);
    Y_R = p2_f_C(2);
    Z_R = p2_f_C(3);
% keyboard
    J_L_i = (1/Z_L)*[1 0 -X_L/Z_L; 0 1 -Y_L/Z_L];
    J_R_i = (1/Z_R)*[1 0 -X_R/Z_R; 0 1 -Y_R/Z_R];% For the second cam

%     H_f_j((2*c_i - 1):2*c_i, :) = J_i*C_CG;
% 
%     H_x_j((2*c_i - 1):2*c_i,12+6*(camStateIndex-1) + 1:12+6*(camStateIndex-1) + 3) = J_i*crossMat(p_f_C);
%     H_x_j((2*c_i - 1):2*c_i,(12+6*(camStateIndex-1) + 4):(12+6*(camStateIndex-1) + 6)) = -J_i*C_CG;
    
    
    
    
    
%     H_f_j((4*c_i - 3):4*c_i-2, :) = J_L_i*C_CG;
%     H_f_j((4*c_i - 1):4*c_i, :) = J_R_i*C_CG; % For the second cam
     
    H1_f_j((2*c_i - 1):2*c_i, :) = J_L_i*C_CG;
    H2_f_j((2*c_i - 1):2*c_i, :) = J_R_i*C2_CG;
    
    
%     H_x_j((4*c_i - 3):(4*c_i-2),12+2*6*(camStateIndex-1) + 1:12+2*6*(camStateIndex-1) + 3) = J_L_i*crossMat(p_f_C);
%     H_x_j((4*c_i - 3):(4*c_i-2),(12+2*6*(camStateIndex-1) + 4):(12+2*6*(camStateIndex-1) + 6)) = -J_L_i*C_CG;
%     % For the second cam
%     H_x_j((4*c_i - 1):4*c_i,12+2*6*(camStateIndex-1) + 1:12+2*6*(camStateIndex-1) + 3) = J_R_i*crossMat(p2_f_C);
%     H_x_j((4*c_i - 1):4*c_i,(12+2*6*(camStateIndex-1) + 4):(12+2*6*(camStateIndex-1) + 6)) = -J_R_i*C2_CG;

% the problem is now from H....

    H1_x_j((2*c_i - 1):2*c_i,12+6*(camStateIndex-1) + 1:12+6*(camStateIndex-1) + 3) = J_L_i*crossMat(p_f_C);
    H1_x_j((2*c_i - 1):2*c_i,(12+6*(camStateIndex-1) + 4):(12+6*(camStateIndex-1) + 6)) = -J_L_i*C_CG;
    % For the second cam
    H2_x_j((2*c_i - 1):2*c_i,6*(camStateIndex-1) + 1:6*(camStateIndex-1) + 3) = J_R_i*crossMat(p2_f_C);
    H2_x_j((2*c_i - 1):2*c_i,(6*(camStateIndex-1) + 4):(6*(camStateIndex-1) + 6)) = -J_R_i*C2_CG;



    c_i = c_i + 1;
end
H11_x_j = [H1_x_j , zeros(2*M,size(H1_x_j,2)-12)];
H22_x_j = [zeros(2*M,size(H1_x_j,2)), H2_x_j];
H_f_j = [H1_f_j ; H2_f_j];
H_x_j = [H11_x_j ; H22_x_j];

A_j = null(H_f_j');
H_o_j = A_j'*H_x_j;

end

