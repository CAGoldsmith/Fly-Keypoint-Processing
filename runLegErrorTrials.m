%Clarissa Goldsmith
%West Virginia University
%11/2/21
clear all
close all

%CHANGE THESE VALUES
% filenameBase = {'201217_bolt-Chr_Fly06_F_T004';'201217_bolt-Chr_Fly07_F_T012';'201218_bolt-Chr_Fly06_M_T011'};
% filenameBase = {'201217_bolt-Chr_Fly06_F_T004'};
toPlot = 1;
filenameBase = {'201218_bolt-Chr_Fly06_M_T011'; '201217_bolt-Chr_Fly03_M_T002'; '201217_bolt-Chr_Fly05_M_T007';...
    '201217_bolt-Chr_Fly06_F_T004'; '201217_bolt-Chr_Fly07_F_T012'};
filepathBase = 'C:\Users\Clarissa G\Videos';
% filepathBase = 'G:\Other computers\NeuroMINT Lab Computer\Videos';
savePathBase = 'C:\Users\Clarissa G\Documents\MATLAB\Fly Keypoint Processing\';
% savePathBase = 'G:\Other computers\NeuroMINT Lab Computer\MATLAB\Fly Keypoint Processing\';

% leg = [1,2,3,4,5,6];
leg = [3,6];
% fixedJoints = [0,0,0,0,0,0,0; 1,1,0,0,0,0,0; 1,1,0,0,0,1,0; 1,1,1,0,0,1,0; 1,1,0,0,1,1,0];
% fixedJoints = [1,0,0,0,0,1,0; 0,1,0,0,0,1,0; 0,0,1,0,0,1,0; 0,0,0,0,0,1,0; 0,0,0,0,1,1,0]; 
% fixedJoints = [0,0,0,0,0,0,0;...
%     1,0,0,0,0,0,0;...
%     1,0,0,0,0,1,0;...
%     0,1,0,0,0,0,0;...
%     0,1,0,0,0,1,0;...
%     0,0,1,0,0,0,0;...
%     0,0,1,0,0,1,0;...
%     1,1,0,0,0,0,0;...
%     1,1,0,0,0,1,0;...
%     1,0,1,0,0,0,0;...
%     1,0,1,0,0,1,0;...
%     0,1,1,0,0,0,0;...
%     0,1,1,0,0,1,0;...
%     1,0,0,0,1,1,0;...
%     0,1,0,0,1,1,0;...
%     0,0,1,0,1,1,0];
fixedJoints = [0,0,0,0,0,0,0; 1,1,0,0,0,1,0];
%%%%%

%In file:   1:6 is RF leg, 7:12 is RM, 13:18 is RH, 19:24 is LF, 25:30 is
%LM, 31:36 is LH
%Order of points is: ThC, CTr, TrF, FTi, TiTar, Tar
legInfo.lowIDs = [1 7 13 19 25 31];
legInfo.highIDs = [6 12 18 24 30 36];
legInfo.leg = {'Right Front'; 'Right Middle'; 'Right Hind'; 'Left Front'; 'Left Middle'; 'Left Hind'};
legInfo.joints = {'ThC1 (pitch)';'ThC2 (yaw)';'ThC3 (roll)';'CTr';'TrF1 (roll)';'TrF2 (pitch)';'FTi';'TiTar'};

for l = 1:length(leg)

    lowID = legInfo.lowIDs(leg(l));
    hiID = legInfo.highIDs(leg(l));
    legName = legInfo.leg{leg(l)};
    [runs,~] = size(fixedJoints);

    for fl = 1:length(filenameBase)

        filenameSplit = split(filenameBase{fl},'_');
        filename = [filenameBase{fl} '_keypoints'];
        filepath = [filepathBase '\' filenameSplit{1} '_' filenameSplit{2} '\' filenameBase{fl}];
        file = [filepath '\' filename '.mat'];
        load(file)

        savePath = [savePathBase filenameBase{fl} '\' legName];

        [frames, joints, dims] = size(keypoints3D); %Determine the size of the file

        keypoints3D(:,:,3) = -keypoints3D(:,:,3); %Flip the z axis so it's right side up
        keypoints3D(:,:,1) = -keypoints3D(:,:,1);



        %Create an array for the position of each joint for each frame
        ThC = [keypoints3D(:,lowID,1) keypoints3D(:,lowID,2) keypoints3D(:,lowID,3)];
        CTr = [keypoints3D(:,lowID+1,1) keypoints3D(:,lowID+1,2) keypoints3D(:,lowID+1,3)];
        TrF = [keypoints3D(:,lowID+2,1) keypoints3D(:,lowID+2,2) keypoints3D(:,lowID+2,3)];
        FTi = [keypoints3D(:,lowID+3,1) keypoints3D(:,lowID+3,2) keypoints3D(:,lowID+3,3)];
        TiTar = [keypoints3D(:,lowID+4,1) keypoints3D(:,lowID+4,2) keypoints3D(:,lowID+4,3)];

        %Create an array of the vectors for each leg segment for each frame
        coxaVec = [CTr(:,1)-ThC(:,1), CTr(:,2)-ThC(:,2), CTr(:,3)-ThC(:,3)];
        trocVec = [TrF(:,1)-CTr(:,1), TrF(:,2)-CTr(:,2), TrF(:,3)-CTr(:,3)];
        femurVec = [FTi(:,1)-TrF(:,1), FTi(:,2)-TrF(:,2), FTi(:,3)-TrF(:,3)];
        tibiaVec = [TiTar(:,1)-FTi(:,1), TiTar(:,2)-FTi(:,2), TiTar(:,3)-FTi(:,3)];

        %Find the lengths of each segment for each frame
        coxaMag = vecnorm(coxaVec,2,2);

        trocMag = vecnorm(trocVec,2,2);

        femurMag = vecnorm(femurVec,2,2);

        tibiaMag = vecnorm(tibiaVec,2,2);

        segMags = [coxaMag, trocMag, femurMag, tibiaMag];


        FTi_start = acosd(dot(femurVec(1,:),tibiaVec(1,:))/(femurMag(1)*tibiaMag(1)))-180;
        ThC2_start =  atand(coxaVec(1,1)/coxaVec(1,3));

        omegaHatCoxa = [0 0 1; 0 0 0; -1 0 0];
        Rx = eye(3) + omegaHatCoxa*sind(-ThC2_start) + omegaHatCoxa^2*(1-cosd(-ThC2_start));
        coxaThC1Plane = (Rx*coxaVec(1,:)')';
        ThC1_start = atand(coxaThC1Plane(2)/coxaThC1Plane(3));
        %We will set the origin of the spatial frame as the ThC joint. Normalize
        %the joint locations in this frame
        CTr = coxaVec;
        TrF = CTr + trocVec;
        FTi = TrF + femurVec;
        TiTar = FTi + tibiaVec;

        for i=1:runs
            [thetas{i},errors{i}] = findMinLegError(leg(l),legInfo,frames,legName,filenameBase{fl},savePath,savePathBase,segMags,CTr,TrF,FTi,TiTar,ThC1_start,ThC2_start,FTi_start,fixedJoints(i,:),toPlot);
            disp(['Condition ' num2str(i) '/' num2str(runs) ' complete'])
        end
        disp([filenameBase{fl} ' complete.']);
    end
    close all
    disp(['Leg ' num2str(l) ' complete.']);
end
