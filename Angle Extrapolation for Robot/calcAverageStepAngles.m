clear all
close all
legNames = {'Left Hind','Right Hind','Left Middle','Right Middle','Left Front','Right Front'};
numFreeJoints = [3 3 3 3 5 5];
stepStartIDs = importdata('201218_stepStartIDs.mat');
for l=1:6
    if l > 4
        % data{l} = importdata(['C:\Users\Clarissa G\Documents\MATLAB\Fly Keypoint Processing\201218_bolt-Chr_Fly06_M_T011\' legNames{l} '\201218_bolt-Chr_Fly06_M_T011_' legNames{l} '_Robot Angles New_ThC2 (yaw), TrF2 (pitch) Fixed.txt']);
        data{l} = importdata(['G:\Other computers\NeuroMINT Lab Computer\MATLAB\Fly Keypoint Processing\201218_bolt-Chr_Fly06_M_T011\' legNames{l} '\201218_bolt-Chr_Fly06_M_T011_' legNames{l} '_Robot Angles New_ThC2 (yaw), TrF2 (pitch) Fixed.txt']);

    else
        % data{l} = importdata(['C:\Users\Clarissa G\Documents\MATLAB\Fly Keypoint Processing\201218_bolt-Chr_Fly06_M_T011\' legNames{l} '\201218_bolt-Chr_Fly06_M_T011_' legNames{l} '_Robot Angles_ThC1 (pitch), ThC2 (yaw), ThC3 (roll), TrF2 (pitch) Fixed.txt']);
        data{l} = importdata(['G:\Other computers\NeuroMINT Lab Computer\MATLAB\Fly Keypoint Processing\201218_bolt-Chr_Fly06_M_T011\' legNames{l} '\201218_bolt-Chr_Fly06_M_T011_' legNames{l} '_Robot Angles_ThC1 (pitch), ThC2 (yaw), ThC3 (roll), TrF2 (pitch) Fixed.txt']);

    end

    figure
    tl_all{l} = tiledlayout(numFreeJoints(l),1);
    if l > 4
        nexttile
        plot(data{l}(:,1))
        title('ThC1')
        xlim([1 length(data{l})])
        nexttile
        plot(data{l}(:,3))
        title('ThC3')
        xlim([1 length(data{l})])
    end
    nexttile
    plot(data{l}(:,4));
    title('CTr')
    xlim([1 length(data{l})])
    hold on
    nexttile
    plot(data{l}(:,5));
    title('TrF')
    xlim([1 length(data{l})])
    hold on
    nexttile
    plot(data{l}(:,7));
    xlim([1 length(data{l})])
    title('FTi')
    hold on
    title(tl_all{l},[legNames{l} ' - All Steps'])

    for j=1:numFreeJoints(l)
        nexttile(j)
        for i=1:length(stepStartIDs{l})
            xline(stepStartIDs{l}(i));
        end
    end

    %Separate data into steps and for each step, interpolate to create the desired number of pts
    stepNum = length(stepStartIDs{l})-1;

    figure
    tl_avg{l} = tiledlayout(numFreeJoints(l),1);

    ptsPerStep{l} = stepStartIDs{l}(2:end)-stepStartIDs{l}(1:end-1);
    maxStepPts = max(ptsPerStep{l});

    for s=1:stepNum
        if l > 4
            ThC1anglesTemp = data{l}(stepStartIDs{l}(s):stepStartIDs{l}(s+1),1);
            ThC3anglesTemp = data{l}(stepStartIDs{l}(s):stepStartIDs{l}(s+1),3);
        end
        TrFanglesTemp = data{l}(stepStartIDs{l}(s):stepStartIDs{l}(s+1),5);
        CTranglesTemp = data{l}(stepStartIDs{l}(s):stepStartIDs{l}(s+1),4);
        FTianglesTemp = data{l}(stepStartIDs{l}(s):stepStartIDs{l}(s+1),7);
        if ptsPerStep{l}(s) < maxStepPts
            if l > 4
                ThC1stepAngles{l}(s,:) = interp1(linspace(0,1,ptsPerStep{l}(s)+1),ThC1anglesTemp,linspace(0,1,maxStepPts+1));
                ThC3stepAngles{l}(s,:) = interp1(linspace(0,1,ptsPerStep{l}(s)+1),ThC3anglesTemp,linspace(0,1,maxStepPts+1));
            end
            TrFstepAngles{l}(s,:) = interp1(linspace(0,1,ptsPerStep{l}(s)+1),TrFanglesTemp,linspace(0,1,maxStepPts+1));
            CTrstepAngles{l}(s,:) = interp1(linspace(0,1,ptsPerStep{l}(s)+1),CTranglesTemp,linspace(0,1,maxStepPts+1));
            FTistepAngles{l}(s,:) = interp1(linspace(0,1,ptsPerStep{l}(s)+1),FTianglesTemp,linspace(0,1,maxStepPts+1));
        else
            if l > 4
                ThC1stepAngles{l}(s,:) = ThC1anglesTemp;
                ThC3stepAngles{l}(s,:) = ThC3anglesTemp;
            end
            TrFstepAngles{l}(s,:) = TrFanglesTemp;
            CTrstepAngles{l}(s,:) = CTranglesTemp;
            FTistepAngles{l}(s,:) = FTianglesTemp;
        end

        if l > 4
            nexttile(1)
            hold on
            plot(ThC1stepAngles{l}(s,:));
            nexttile(2)
            hold on
            plot(ThC3stepAngles{l}(s,:));
            nexttile(3)
            hold on
            plot(CTrstepAngles{l}(s,:))
            nexttile(4)
            hold on
            plot(TrFstepAngles{l}(s,:))
            nexttile(5)
            hold on
            plot(FTistepAngles{l}(s,:))
        else
            nexttile(1)
            hold on
            plot(CTrstepAngles{l}(s,:))
            nexttile(2)
            hold on
            plot(TrFstepAngles{l}(s,:))
            nexttile(3)
            hold on
            plot(FTistepAngles{l}(s,:))
        end
    end

    %Find the mean angles for a step
    for n=1:length(TrFstepAngles{l})
        if l > 4
            ThC1avgStep{l}(n) = mean(ThC1stepAngles{l}(:,n));
            ThC3avgStep{l}(n) = mean(ThC3stepAngles{l}(:,n));
        end
        TrFavgStep{l}(n) = mean(TrFstepAngles{l}(:,n));
        CTravgStep{l}(n) = mean(CTrstepAngles{l}(:,n));
        FTiavgStep{l}(n) = mean(FTistepAngles{l}(:,n));
    end

    if l > 4
        nexttile(1)
        plot(ThC1avgStep{l},'r','LineWidth',2)
        xlim([1 length(CTravgStep{l})])
        title('ThC1')
        nexttile(2)
        plot(ThC3avgStep{l},'r','LineWidth',2)
        xlim([1 length(CTravgStep{l})])
        title('ThC3')
        nexttile(3)
        plot(CTravgStep{l},'r','LineWidth',2)
        xlim([1 length(CTravgStep{l})])
        title('CTr')
        nexttile(4)
        plot(TrFavgStep{l},'r','LineWidth',2)
        xlim([1 length(CTravgStep{l})])
        title('TrF')
        nexttile(5)
        plot(FTiavgStep{l},'r','LineWidth',2)
        title('FTi')
        xlim([1 length(CTravgStep{l})])
    else
        nexttile(1)
        plot(CTravgStep{l},'r','LineWidth',2)
        xlim([1 length(CTravgStep{l})])
        title('CTr')
        nexttile(2)
        plot(TrFavgStep{l},'r','LineWidth',2)
        xlim([1 length(CTravgStep{l})])
        title('TrF')
        nexttile(3)
        plot(FTiavgStep{l},'r','LineWidth',2)
        title('FTi')
        xlim([1 length(CTravgStep{l})])
    end
    title(tl_avg{l},[legNames{l} ' - Average Step'])

    %Interpolate each joint's angles to a number of pts that the Design Drosophibot code is expecting
    ddPoints = 102;
    if l > 4
        ThC1avgStep{l} = interp1(linspace(0,1,length(ThC1avgStep{l})),ThC1avgStep{l},linspace(0,1,ddPoints));
        ThC3avgStep{l} = interp1(linspace(0,1,length(ThC3avgStep{l})),ThC3avgStep{l},linspace(0,1,ddPoints));
    end
    TrFavgStep{l} = interp1(linspace(0,1,length(TrFavgStep{l})),TrFavgStep{l},linspace(0,1,ddPoints));
    CTravgStep{l} = interp1(linspace(0,1,length(CTravgStep{l})),CTravgStep{l},linspace(0,1,ddPoints));
    FTiavgStep{l} = interp1(linspace(0,1,length(FTiavgStep{l})),FTiavgStep{l},linspace(0,1,ddPoints));

    if l > 4
        ballJointAngles{l} = [ThC1avgStep{l}; ThC3avgStep{l}; CTravgStep{l}; TrFavgStep{l}; FTiavgStep{l}];
    else
        ballJointAngles{l} = [CTravgStep{l}; TrFavgStep{l}; FTiavgStep{l}];
    end
end



% save("C:\Users\Clarissa G\Documents\MATLAB\Fly Keypoint Processing\Angle Extrapolation for Robot\ballJointAngles.mat",'ballJointAngles');
save("G:\Other computers\NeuroMINT Lab Computer\MATLAB\Fly Keypoint Processing\Angle Extrapolation for Robot\ballJointAngles.mat",'ballJointAngles');


%%
l=3;
CTrRad = CTravgStep{l}*pi/180;
TrFRad = TrFavgStep{l}*pi/180;
FTiRad = FTiavgStep{l}*pi/180;
CTrmidVal = mean(CTrRad);
TrFmidVal = mean(TrFRad);
for i=1:length(CTrRad)
        CTrRadFlipped(i) = CTrmidVal + (CTrmidVal-CTrRad(i));
        TrFRadFlipped(i) = TrFmidVal + (TrFmidVal-TrFRad(i));
end
FTiRadFlipped = -FTiRad;



ballJointAnglesModded{l} = [CTrRadFlipped; TrFRad; FTiRadFlipped];
save("G:\Other computers\NeuroMINT Lab Computer\MATLAB\Fly Keypoint Processing\Angle Extrapolation for Robot\ballJointAnglesModded.mat",'ballJointAnglesModded');


