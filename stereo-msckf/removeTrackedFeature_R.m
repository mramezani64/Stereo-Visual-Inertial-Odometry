function  [updatedMsckfState, featCamStates_R, camStateIndices_R] = removeTrackedFeature_R(msckfState, featureId)
%REMOVETRACKEDFEATURE Remove tracked feature from camStates and extract all
%camera states that include it

    updatedCamStates_R = msckfState.camStates_R;
    featCamStates_R = {};
    camStateIndices_R = [];
    for c_i = 1:length(updatedCamStates_R)
        featIdx = find(featureId == updatedCamStates_R{c_i}.trackedFeatureIds_R);
        if ~isempty(featIdx)
            updatedCamStates_R{c_i}.trackedFeatureIds_R(featIdx) = [];
            camStateIndices_R(end + 1) = c_i;
            featCamStates_R{end +1} = updatedCamStates_R{c_i};
        end
    end
    
    updatedMsckfState = msckfState;
    updatedMsckfState.camStates_R = updatedCamStates_R;
end