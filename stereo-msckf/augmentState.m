function msckfState_aug = augmentState(msckfState, camera, state_k)
% Augments the MSCKF state with a new camera pose    

    C_IG = quatToRotMat(msckfState.imuState.q_IG);
    
    % Compute camera pose from current IMU pose
    q_CG = quatLeftComp(camera.q_CI) * msckfState.imuState.q_IG;
    p_C_G = msckfState.imuState.p_I_G + C_IG' * camera.p_C_I;
    
    q2_CG = quatLeftComp(camera.q2_CI) * msckfState.imuState.q_IG;
    p2_C_G = msckfState.imuState.p_I_G + C_IG' * camera.p2_C_I;
%     if state_k==3
% keyboard
%     end
    % Build MSCKF covariance matrix
    P = [msckfState.imuCovar, msckfState.imuCamCovar, msckfState.imuCamCovar;
        msckfState.imuCamCovar', msckfState.camCovar, msckfState.camCovar; 
        msckfState.imuCamCovar', msckfState.camCovar', msckfState.camCovar]; % the covariance for 3 sensors
    
%     P = [msckfState.imuCovar, msckfState.imuCamCovar;
%         msckfState.imuCamCovar', msckfState.camCovar]; % The covariance for 2 seperate sensors
    
    % Camera state Jacobian
    J = calcJ(camera, msckfState.imuState, msckfState.camStates_L);
    
    N = size(msckfState.camStates_L,2);
    
    tempMat = [eye(12+2*6*N); J];
%     size(P),size(J),N
    
    % Augment the MSCKF covariance matrix
    P_aug = tempMat * P * tempMat';
%     size(P_aug)
    
    % Break everything into appropriate structs
    msckfState_aug = msckfState;
    msckfState_aug.camStates_L{N+1}.p_C_G = p_C_G;
    msckfState_aug.camStates_L{N+1}.q_CG = q_CG;
    msckfState_aug.camStates_R{N+1}.p2_C_G = p2_C_G; % For the second cam
    msckfState_aug.camStates_R{N+1}.q2_CG = q2_CG; % For the second cam
    msckfState_aug.camStates_L{N+1}.state_k = state_k;
    msckfState_aug.camStates_R{N+1}.state_k = state_k;
    msckfState_aug.camStates_L{N+1}.trackedFeatureIds_L = [];
    msckfState_aug.camStates_R{N+1}.trackedFeatureIds_R = [];
    msckfState_aug.imuCovar = P_aug(1:12,1:12);
    msckfState_aug.camCovar = P_aug(13+6*(N+1):end,13+6*(N+1):end);
    msckfState_aug.imuCamCovar = P_aug(1:12, 13+6*(N+1):end);
end