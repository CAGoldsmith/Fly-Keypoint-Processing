clear all

% videoLocs = {'201218_bolt-Chr\201218_bolt-Chr_Fly06_M_T011'};
% videoVarFileNames = {'201218_bolt-Chr_Fly06_M_T011.mat'};
videoLocs = {'201218_bolt-Chr\201218_bolt-Chr_Fly06_M_T011',...
    '201217_bolt-Chr\201217_bolt-Chr_Fly03_M_T002',...
    '201217_bolt-Chr\201217_bolt-Chr_Fly05_M_T007',...
    '201217_bolt-Chr\201217_bolt-Chr_Fly07_F_T012'};
videoVarFileNames = {'201218_bolt-Chr_Fly06_M_T011.mat',...
    '201217_bolt-Chr_Fly03_M_T002.mat',...
    '201217_bolt-Chr_Fly05_M_T007.mat',...
    '201217_bolt-Chr_Fly07_F_T012.mat'};
startIDVarNames = {'201218_stepStartIDs.mat','201217_T002_stepStartIDs.mat','201217_T007_stepStartIDs.mat',...
    '201217_T012_stepStartIDs.mat'};
trials = 4;
TarTipids = [36, 18, 30, 12, 24, 6];
ThCids = [31, 13, 25, 7, 19, 1];
legNames = {'Left Hind','Right Hind','Left Middle','Right Middle','Left Front','Right Front'};

rED = [-0.0357;.213;-.335];
% rED = [0;.213;-.335];
ThCLoc = rED;
R = 3;

for tr = 1:trials
    load(['C:\Users\Clarissa G\Videos\' videoLocs{tr} '\' videoVarFileNames{tr}],'keypoints3D');
    % load(['G:\Other computers\NeuroMINT Lab Computer\Videos\' videoLocs{tr} '\' videoVarFileNames{tr}],'keypoints3D');
    startIDs = load(['C:\Users\Clarissa G\Documents\MATLAB\Fly Keypoint Processing\Angle Extrapolation for Robot\' startIDVarNames{tr}]);
    % startIDs = load(['G:\Other computers\NeuroMINT Lab Computer\MATLAB\Fly Keypoint Processing\Angle Extrapolation for Robot\' startIDVarNames{tr}]);
    stepStartIDs{tr} = startIDs.stepStartIDs;

    keypoints3D(:,:,3) = -keypoints3D(:,:,3); %Flip the z axis so it's right side up
    keypoints3D(:,:,1) = -keypoints3D(:,:,1);
    keypoints{tr} = keypoints3D;

    hStepAvg = [];
    for s=1:length(stepStartIDs{tr}{3})-1
        for i=1:10
            coxaVec = squeeze(keypoints{tr}(stepStartIDs{tr}{3}(s)+(i-1),ThCids(3)+1,:) - keypoints{tr}(stepStartIDs{tr}{3}(s)+(i-1),ThCids(3),:));
            CTrLoc = ThCLoc + coxaVec;
            trochVec = squeeze(keypoints{tr}(stepStartIDs{tr}{3}(s)+(i-1),ThCids(3)+2,:) - keypoints{tr}(stepStartIDs{tr}{3}(s)+(i-1),ThCids(3)+1,:));
            TrFLoc = CTrLoc + trochVec;
            femVec = squeeze(keypoints{tr}(stepStartIDs{tr}{3}(s)+(i-1),ThCids(3)+3,:) - keypoints{tr}(stepStartIDs{tr}{3}(s)+(i-1),ThCids(3)+2,:));
            FTiLoc = TrFLoc + femVec;
            tibVec = squeeze(keypoints{tr}(stepStartIDs{tr}{3}(s)+(i-1),ThCids(3)+4,:) - keypoints{tr}(stepStartIDs{tr}{3}(s)+(i-1),ThCids(3)+3,:));
            TiTarLoc = FTiLoc + tibVec;
            tarVec = squeeze(keypoints{tr}(stepStartIDs{tr}{3}(s)+(i-1),TarTipids(3),:) - keypoints{tr}(stepStartIDs{tr}{3}(s)+(i-1),ThCids(3)+4,:));
            tarTipLoc = TiTarLoc + tarVec;
            legVec{tr} = [ThCLoc CTrLoc TrFLoc FTiLoc TiTarLoc tarTipLoc];

            rDA = tarTipLoc - ThCLoc;

            AOx = -tarTipLoc(1);
            AOy = -tarTipLoc(2);
            AOz = -sqrt(R^2 - tarTipLoc(1)^2 - tarTipLoc(2)^2);

            rEO = rED + rDA + [AOx;AOy;AOz];
            h{tr}(s,i) = -rEO(3);
        end
        hStepAvg(s) = mean(h{tr}(s,:));
    end
    hTrialAvg(tr) = mean(hStepAvg);
    hTrialPerc(tr) = hTrialAvg(tr)/norm(tibVec);
end

hAvg = mean(hTrialAvg);
hAvgPerc = mean(hTrialPerc);



