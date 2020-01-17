function  [updatedMsckfState, featCamStates_L, camStateIndices_L] = removeTrackedFeature_L(msckfState, featureId)
%REMOVETRACKEDFEATURE Remove tracked feature from camStates and extract all
%camera states that include it

    updatedCamStates_L = msckfState.camStates_L;
    featCamStates_L = {};
    camStateIndices_L = [];
    for c_i = 1:length(updatedCamStates_L)
        featIdx = find(featureId == updatedCamStates_L{c_i}.trackedFeatureIds_L);
        if ~isempty(featIdx)
            updatedCamStates_L{c_i}.trackedFeatureIds_L(featIdx) = [];
            camStateIndices_L(end + 1) = c_i;
            featCamStates_L{end +1} = updatedCamStates_L{c_i};
        end
    end
    
    updatedMsckfState = msckfState;
    updatedMsckfState.camStates_L = updatedCamStates_L;
end

