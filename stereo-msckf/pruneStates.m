function [ prunedMsckfState, deletedCamStates_L, deletedCamStates_R ] = pruneStates( msckfState )
%PRUNESTATES Prunes any states that have no tracked features and updates
%covariances
    
    prunedMsckfState.imuState = msckfState.imuState;
    prunedMsckfState.imuCovar = msckfState.imuCovar;
    
    %Find all camStates with no tracked landmarks    
    deleteIdx_L = [];
    deleteIdx_R = [];
    for c_i = 1:length(msckfState.camStates_L)
        if isempty(msckfState.camStates_L{c_i}.trackedFeatureIds_L)
            deleteIdx_L(end+1) = c_i;
        end
    end
    % For the second cam as well
    for c_i = 1:length(msckfState.camStates_R)
        if isempty(msckfState.camStates_R{c_i}.trackedFeatureIds_R)
            deleteIdx_R(end+1) = c_i;
        end
    end
    
    
    %Prune the damn states! % This damn state is worse when 2 cams exist
    
    deletedCamStates_L = msckfState.camStates_L(deleteIdx_L);
    deletedCamStates_R = msckfState.camStates_R(deleteIdx_R); % For the second cam
%     deletedCamStates = [deletedCamStates_L, deletedCamStates_R];
    prunedMsckfState.camStates_L = removeCells(msckfState.camStates_L, deleteIdx_L);
    prunedMsckfState.camStates_R = removeCells(msckfState.camStates_R, deleteIdx_R); % For the second cam
%     for i=1:length(prunedMsckfState.camStates_L)
%     prunedMsckfState.camStates_L(2*i-1:2*i) = [prunedMsckfState.camStates_L(i)];
%     end
%     for i=1:length(prunedMsckfState.camStates_R)
%     prunedMsckfState.camStates_R(2*i-1:2*i) = [prunedMsckfState.camStates_R(i)];
%     end
%     prunedMsckfState.camStates = [prunedMsckfState.camStates_L,prunedMsckfState.camStates_R];
    
    statesIdx = 1:size(msckfState.camCovar,1);
    keepCovarMask = true(1, numel(statesIdx));
    for dIdx = deleteIdx_L
        keepCovarMask(6*dIdx - 5:6*dIdx) = false(6,1); % For a stereo vision
    end
%     keyboard
    keepCovarIdx = statesIdx(keepCovarMask);
    deleteCovarIdx = statesIdx(~keepCovarMask);
    
    prunedMsckfState.camCovar = msckfState.camCovar(keepCovarIdx, keepCovarIdx);
    %Keep rows, prune columns of upper right covariance matrix
    prunedMsckfState.imuCamCovar = msckfState.imuCamCovar(:, keepCovarIdx);
    
    deletedCamCovar = msckfState.camCovar(deleteCovarIdx, deleteCovarIdx);
    deletedCamSigma = sqrt(diag(deletedCamCovar));
    % Grab the variances of the deleted states for plotting
    for c_i = 1:size(deletedCamStates_L, 2)
        deletedCamStates_L{c_i}.sigma = deletedCamSigma(6*c_i - 5 : 6*c_i);
    end
end

