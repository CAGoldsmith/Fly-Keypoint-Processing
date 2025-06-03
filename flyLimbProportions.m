%Clarissa Goldsmith
%West Virginia University
%12/10/21

%Get ratios for each of the leg segments/the distances between them on the
%body

%In this code, x runs lateral to the body, y runs through central ant/post
%axis of body, and z goes ventral-dorsal
clear all

filenameBase = {'201217_bolt-Chr_Fly03_M_T002'; '201217_bolt-Chr_Fly05_M_T007';...
    '201218_bolt-Chr_Fly06_M_T011';'201217_bolt-Chr_Fly07_F_T012';'201217_bolt-Chr_Fly06_F_T004'};
% filepathBase = 'C:\Users\Clarissa G\Videos';
filepathBase = 'G:\Other computers\NeuroMINT Lab Computer\Videos';
legInfo.lowIDs = [1 7 13 19 25 31];
legInfo.highIDs = [6 12 18 24 30 36];
legInfo.leg = {'Right Front'; 'Right Middle'; 'Right Hind'; 'Left Front'; 'Left Middle'; 'Left Hind'};
legInfo.joints = {'ThC1 (pitch)';'ThC2 (yaw)';'ThC3 (roll)';'CTr';'TrF1 (roll)';'TrF2 (pitch)';'FTi';'TiTar'};


for fl = 1:length(filenameBase)
    filenameSplit = split(filenameBase{fl},'_');
    filename = [filenameBase{fl} '_keypoints'];
    filepath = [filepathBase '\' filenameSplit{1} '_' filenameSplit{2} '\' filenameBase{fl}];
    file = [filepath '\' filename '.mat'];
    load(file)
    keypoints3D(:,:,3) = -keypoints3D(:,:,3); %Flip the z axis so it's right side up
    keypoints3D(:,:,1) = -keypoints3D(:,:,1);
    
    for L=1:6
        lowID = legInfo.lowIDs(L);
        hiID = legInfo.highIDs(L);
        legName = legInfo.leg{L};
        
        ThC = [keypoints3D(:,lowID,1) keypoints3D(:,lowID,2) keypoints3D(:,lowID,3)]';
        ThCpos(:,L) = mean(ThC,2);
        CTr = [keypoints3D(:,lowID+1,1) keypoints3D(:,lowID+1,2) keypoints3D(:,lowID+1,3)]';
        TrF = [keypoints3D(:,lowID+2,1) keypoints3D(:,lowID+2,2) keypoints3D(:,lowID+2,3)]';
        FTi = [keypoints3D(:,lowID+3,1) keypoints3D(:,lowID+3,2) keypoints3D(:,lowID+3,3)]';
        TiTar = [keypoints3D(:,lowID+4,1) keypoints3D(:,lowID+4,2) keypoints3D(:,lowID+4,3)]';
        TarTip{L} = [keypoints3D(:,lowID+5,1) keypoints3D(:,lowID+5,2) keypoints3D(:,lowID+5,3)]';
        TarTipDiff{L} = TarTip{L}(:,1:end-1)-TarTip{L}(:,2:end);

        TarTipCheck = TarTipDiff{L}(2,:) < 0;
        j = 1;
        for i=1:length(TarTipCheck)
            if TarTipCheck(i) == 0
                TarTipSt{L}(:,j) = TarTip{L}(:,i);
                j = j+1;
            end
        end
        avgTarTipSt{L} = mean(TarTipSt{L},2);

        RPThCTarDist{L}(:,fl) = ThCpos(:,L)-avgTarTipSt{L};
        RPThCTarDist{L}(1,fl) = abs(RPThCTarDist{L}(1,fl));

        StrLength(fl,L) = max(TarTip{L}(2,:))-min(TarTip{L}(2,:));

        %Create an array of the vectors for each leg segment for each frame
        coxaVec = CTr - ThC;
        trocVec = TrF - CTr;
        femurVec = FTi - TrF;
        tibiaVec = TiTar - FTi;
        tarsVec = TarTip{L} - TiTar;
        
        %Find the average magnitude of each segment throughout an entire
        %trial video
        coxaMag(fl,L) = mean(vecnorm(coxaVec));
        trocMag(fl,L) = mean(vecnorm(trocVec));
        femurMag(fl,L) = mean(vecnorm(femurVec));
        tibiaMag(fl,L) = mean(vecnorm(tibiaVec));
        tarsMag(fl,L) = mean(vecnorm(tarsVec));
       

    end  
    for i=1:3
        stWidth{i}(:,fl) =  norm(avgTarTipSt{i} - avgTarTipSt{i+3});
    end
   
    
    %Find what the distance between the ThC joints in each dimension is on the body for
    %each side this trial
    FMThCDist(:,fl) = mean([abs(ThCpos(:,1)-ThCpos(:,2)) abs(ThCpos(:,4)-ThCpos(:,5))],2);
    
    MHThCDist(:,fl) = mean([abs(ThCpos(:,2)-ThCpos(:,3)) abs(ThCpos(:,5)-ThCpos(:,6))],2);

    
    %Find the width between the pairs of ThC joints for this trial
    ThCwidth(1,fl) = abs(ThCpos(1,1)-ThCpos(1,4));
    ThCwidth(2,fl) = abs(ThCpos(1,2)-ThCpos(1,5));
    ThCwidth(3,fl) = abs(ThCpos(1,3)-ThCpos(1,6));

    %Find the thorax height by subtracting the z pos of the posterior
    %scutellum apex from the z pos of the middle ThC
    scutApexPos(:,fl) = mean([keypoints3D(:,37,1) keypoints3D(:,37,2) keypoints3D(:,37,3)]',2);
    thoraxDims(3,fl) = scutApexPos(3) - mean([ThCpos(3,3); ThCpos(3,4)]);
    %Find the thorax width by substracting the x pos of the two wing points
    wingHingeRPos(:,fl) = mean([keypoints3D(:,39,1) keypoints3D(:,39,2) keypoints3D(:,39,3)]',2);
    wingHingeLPos(:,fl) = mean([keypoints3D(:,38,1) keypoints3D(:,38,2) keypoints3D(:,38,3)]',2);
    wingHingePos(3,fl) = mean([wingHingeLPos(3,fl); wingHingeRPos(3,fl)]) - mean([ThCpos(3,1); ThCpos(3,2)]);
    wingHingePos(1,fl) = mean([wingHingeLPos(1,fl); wingHingeRPos(1,fl)]) - mean([ThCpos(1,1); ThCpos(1,2)]);
    wingHingePos(2,fl) = mean([wingHingeLPos(2,fl); wingHingeRPos(2,fl)]) - mean([ThCpos(2,1); ThCpos(2,2)]);
    wingW(fl) = abs(wingHingeLPos(1,fl) - wingHingeRPos(1,fl));
    thoraxDims(1,fl) = wingHingeRPos(1,fl) - wingHingeLPos(1,fl);
    %Find the thorax length by subtracting the front ThC y pos from the
    %posterior scutellum apex y pos
    thoraxDims(2,fl) = mean([ThCpos(2,1); ThCpos(2,2)]) - scutApexPos(2,fl);

    antennaRPos(:,fl) = mean([keypoints3D(:,41,1) keypoints3D(:,41,2) keypoints3D(:,41,3)]',2); 
    antennaLPos(:,fl) = mean([keypoints3D(:,40,1) keypoints3D(:,40,2) keypoints3D(:,40,3)]',2); 
    headL(fl) = mean([antennaRPos(2,fl); antennaLPos(2,fl)]) - mean([ThCpos(2,1); ThCpos(2,2)]);
end

for i=1:3
    avgStWidth(:,i) = mean(stWidth{i},2);
    RPThCTarL(:,i) = mean([RPThCTarDist{i} RPThCTarDist{i+3}],2);
end

avgStrLength = mean(mean(StrLength),2);
wingHingePosAvg = mean(wingHingePos,2);
wingWAvg = mean(wingW);

ThCWidthFront = mean(ThCwidth(1,:),2);
ThCWidthMid = mean(ThCwidth(2,:),2);
ThCWidthHind = mean(ThCwidth(3,:),2);

FrontMidLength = mean(FMThCDist(2,:),2);
MidHindLength = mean(MHThCDist(2,:),2);
bodyLength = FrontMidLength + MidHindLength;

frontLimbLengths = [mean([coxaMag(:,1);coxaMag(:,4)]),mean([trocMag(:,1);trocMag(:,4)]),mean([femurMag(:,1);femurMag(:,4)]),...
    mean([tibiaMag(:,1);tibiaMag(:,4)]), mean([tarsMag(:,1);tarsMag(:,4)])];

midLimbLengths = [mean([coxaMag(:,2);coxaMag(:,5)]),mean([trocMag(:,2);trocMag(:,5)]),mean([femurMag(:,2);femurMag(:,5)]),...
    mean([tibiaMag(:,2);tibiaMag(:,5)]), mean([tarsMag(:,2);tarsMag(:,5)])];

hindLimbLengths = [mean([coxaMag(:,3);coxaMag(:,6)]),mean([trocMag(:,3);trocMag(:,6)]),mean([femurMag(:,3);femurMag(:,6)]),...
    mean([tibiaMag(:,3);tibiaMag(:,6)]), mean([tarsMag(:,3);tarsMag(:,6)])];

avgThoraxDims = mean(thoraxDims,2);

%% Calculate how big the robot's parts should be based on scaling

%CHANGE THIS:
ratioVar = midLimbLengths(3); %Middle leg femur
varLength = 10; %cm
% ratioVar = midLimbLengths(2);
% varLength = 28; %mm


avgStWidthRat = avgStWidth./ratioVar;

ThCWidthFrat = ThCWidthFront/ratioVar;
ThCWidthMrat = ThCWidthMid/ratioVar;
ThCWidthHrat = ThCWidthHind/ratioVar;
bodyLFMrat = FrontMidLength/ratioVar;
bodyLMHrat = MidHindLength/ratioVar;

frontLimbRats = frontLimbLengths/ratioVar;
midLimbRats = midLimbLengths/ratioVar;
hindLimbRats = hindLimbLengths/ratioVar;

IdRobotStWidth = avgStWidthRat*varLength;

IdRobotThCWidthF = varLength*ThCWidthFrat;
IdRobotThCWidthM = varLength*ThCWidthMrat;
IdRobotThCWidthH = varLength*ThCWidthHrat;
IdRobotBodyLFM = varLength*bodyLFMrat;
IdRobotBodyLMH = varLength*bodyLMHrat;

IdrobotFrontLimbL = varLength*frontLimbRats;
IdrobotMidLimbL = varLength*midLimbRats;
IdrobotHindLimbL = varLength*hindLimbRats;

IdrobotStrLength = avgStrLength/ratioVar*varLength;

IdrobotThCTarDist = RPThCTarL./ratioVar.*varLength;

%There will be some variability between what would be biologically exact
%and what is robotically feasible. Presently, the biggest discrepancies are
%the robot width and the distance between leg pair
%The hind ThC width determines the difference for the rest of them:
ActRobotThCWidthH = 113.14/10; %cm
ThCWidthDiff = ActRobotThCWidthH-IdRobotThCWidthH;
ActRobotThCWidthM = IdRobotThCWidthM + ThCWidthDiff;
ActRobotThCWidthF = IdRobotThCWidthF + ThCWidthDiff;

ActRobotStWidth = IdRobotStWidth + ThCWidthDiff;

%Scale this into meters and divide in half to get where the feet pos should
%be in the Animatlab file
robotFootStDist = ActRobotStWidth./200;

%The distance between the middle and hind leg pairs determines the
%difference for the middle and front leg pairs
ActRobotBodyLMH = 7.78; %cm
BodyLDiff = ActRobotBodyLMH - IdRobotBodyLMH;
ActRobotBodyLFM = IdRobotBodyLFM + BodyLDiff;

%Save all of these variables in a file to open later
save('flyLimbProportions10cmFemur.mat')

