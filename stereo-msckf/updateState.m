function msckfState_up = updateState(msckfState, deltaX)
% Updates MSCKF state with deltaX

    % Initialize updated state with current state
    msckfState_up = msckfState;

    % Update IMU State
    deltatheta_IG = deltaX(1:3);
    deltab_g = deltaX(4:6);
    deltab_v = deltaX(7:9);
    deltap_I_G = deltaX(10:12);
    
    deltaq_IG = buildUpdateQuat(deltatheta_IG);
    
    msckfState_up.imuState.q_IG = quatLeftComp(deltaq_IG) * msckfState.imuState.q_IG;
    msckfState_up.imuState.b_g = msckfState.imuState.b_g + deltab_g;
    msckfState_up.imuState.b_v = msckfState.imuState.b_v + deltab_v;
    msckfState_up.imuState.p_I_G = msckfState.imuState.p_I_G + deltap_I_G;
    
    % Update camera states , for two cameras we have two camstates
     % Update camera states
%     for i = 1:size(msckfState.camStates, 2)
%         qStart = 12 + 6*(i-1) + 1;
%         pStart = qStart+3;
%         
%         deltatheta_CG = deltaX(qStart:qStart+2);
%         deltap_C_G = deltaX(pStart:pStart+2);
%         
%         deltaq_CG = buildUpdateQuat(deltatheta_CG);
%         
%         msckfState_up.camStates{i}.q_CG = quatLeftComp(deltaq_CG) * msckfState.camStates{i}.q_CG;
%         msckfState_up.camStates{i}.p_C_G = msckfState.camStates{i}.p_C_G + deltap_C_G;
%     end
    
    stateNum = size(msckfState.camStates_L, 2);
    for i = 1:size(msckfState.camStates_L, 2)
%         qStart = 12 + 2*6*(i-1) + 1;
        qStart = 12 + 6*(i-1) + 1;
        
        pStart = qStart+3;
        
        deltatheta_CG = deltaX(qStart:qStart+2);
        deltatheta2_CG = deltaX(qStart+6*stateNum:qStart+6*stateNum+2); % For the second cam
        
        deltap_C_G = deltaX(pStart:pStart+2);
        deltap2_C_G = deltaX(pStart+6*stateNum:pStart+6*stateNum+2); % For the second cam
        
        deltaq_CG = buildUpdateQuat(deltatheta_CG);
        deltaq2_CG = buildUpdateQuat(deltatheta2_CG); % For the second cam
        
        
        msckfState_up.camStates_L{i}.q_CG = quatLeftComp(deltaq_CG) * msckfState.camStates_L{i}.q_CG;
        msckfState_up.camStates_L{i}.p_C_G = msckfState.camStates_L{i}.p_C_G + deltap_C_G;
        % FOr the second cam
        msckfState_up.camStates_R{i}.q2_CG = quatLeftComp(deltaq2_CG) * msckfState.camStates_R{i}.q2_CG;
        msckfState_up.camStates_R{i}.p2_C_G = msckfState.camStates_R{i}.p2_C_G + deltap2_C_G;
    end
    
end