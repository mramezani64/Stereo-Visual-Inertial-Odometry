function [msckfState, featureTracks_L, trackedFeatureIds_L, featureTracks_R, trackedFeatureIds_R] = initializeMSCKF(firstImuState, firstMeasurements, camera, state_k, noiseParams)
%INITIALIZEMSCKF Initialize the MSCKF with tracked features and ground
%truth


%Compute the first state
firstImuState.b_g = zeros(3,1);
firstImuState.b_v = zeros(3,1);
msckfState.imuState = firstImuState;
msckfState.imuCovar = noiseParams.initialIMUCovar;
msckfState.camCovar = [];
msckfState.imuCamCovar = [];
msckfState.camStates_L = {};
msckfState.camStates_R = {}; % for the second cam
msckfState = augmentState(msckfState, camera, state_k);

%Compute all of the relevant feature tracks
featureTracks_L = {};
featureTracks_R = {};
trackedFeatureIds_L = [];
trackedFeatureIds_R = [];


% For left camera

 for featureId = 1:size(firstMeasurements.y_L,2)
        meas_k_L = firstMeasurements.y_L(:, featureId);
        meas_k_R = firstMeasurements.y_R(:, featureId);
        if ~isnan(meas_k_L(1,1))
                %Track new feature
                track_L.featureId = featureId;
                track_R.featureId = featureId; % for the second cam
                track_L.observations_L = meas_k_L;
                track_R.observations_R = meas_k_R; % for the second cam
                featureTracks_L{end+1} = track_L;
                featureTracks_R{end+1} = track_R; % for the second cam
                trackedFeatureIds_L(end+1) = featureId;
                trackedFeatureIds_R(end+1) = featureId; % for the second cam
                %Add observation to current camera
                msckfState.camStates_L{end}.trackedFeatureIds_L(end+1) = featureId;
                msckfState.camStates_R{end}.trackedFeatureIds_R(end+1) = featureId; % for the second cam
                
        end
 end
 
 end