function J = calcJ(camera, imuState_k, camStates_k)
% Jacobian of feature observations w.r.t. feature locations
% In stereo vision, two camera pose estimates are appended to the state
% vector
% p_C2_C1 = - camera.p_C1_C2;
% C_C2_C1 = camera.C_C2_C1;
    C_CI = quatToRotMat(camera.q_CI);
    C2_CI = quatToRotMat(camera.q2_CI); % for the second camera
    C_IG = quatToRotMat(imuState_k.q_IG);

%     J = zeros(6, 12 + 6*size(camStates_k,2));
%     J(1:3,1:3) = C_CI;
%     J(4:6,1:3) = crossMat(C_IG' * camera.p_C_I);
%     J(4:6,10:12) = eye(3);

% we need to change the Jacobian matrix according to 3 sesors:

J = zeros(2*6, 12 + 2*6*size(camStates_k,2));
J(1:3,1:3) = C_CI;
J(4:6,1:3) = crossMat(C_IG' * camera.p_C_I);
J(4:6,10:12) = eye(3);
% J(7:9,1:3) =  C_C2_C1*C_CI;
J(7:9,1:3) =  C2_CI;
% J(10:12,1:3) = crossMat(C_IG' * (camera.p_C_I + C_CI*p_C2_C1));
J(10:12,1:3) = crossMat(C_IG' * camera.p2_C_I );
J(10:12,10:12) = eye(3);
end

% new params
% C_C2C1, P_C2_C1