clear all
close all
%% Define the specific angle data to plot

% legNames = {'Left Hind','Right Hind','Left Middle','Right Middle','Left Front','Right Front'};
legNames = {'Right Middle'}; % Define the specific leg(s) to plot 
trialNames = {'201217_bolt-Chr_Fly03_M_T002','201217_bolt-Chr_Fly05_M_T007','201217_bolt-Chr_Fly06_F_T004',...
    '201217_bolt-Chr_Fly07_F_T012','201218_bolt-Chr_Fly06_M_T011'}; %Define the specific trials to plot
filePathBase = [pwd '\']; %Assuming the current filepath is in the 'Fly Keypoint Processing' folder, pull that filepath as the base from which to pull up the angle files

%Set the names of the specific conditions from which to plot
conds = {'ThC1 (pitch), ThC2 (yaw), TrF2 (pitch) Fixed'}; %For middle and hind legs
% conds = {'ThC2 (yaw), TrF2 (pitch) Fixed'}; %For front legs

jointNames = {'ThC1', 'ThC2', 'ThC3', 'CTr', 'TrF1', 'TrF2','FTi'};

%% Plot the angle data
for l=1:length(legNames)
    for t=1:length(trialNames)
        if l < 5
            if l < 3
                c = 1;
            else
                c = 2;
            end
        else
            c = 3;
        end
        fileName = [trialNames{t} '_' legNames{l} '_Robot Angles New_' conds{c} '.mat'];
        load([filePathBase '\' trialNames{t} '\' legNames{l} '\' fileName])

        for j=1:7
                if mean(thetas(:,j)) > 100
                    if mean(thetas(:,j)) > 275
                        thetas(:,j) = thetas(:,j) - 360;
                    else
                        thetas(:,j) = thetas(:,j) - 180;
                    end
                elseif mean(thetas(:,j)) < -100
                    if mean(thetas(:,j)) < -275
                        thetas(:,j) = thetas(:,j) + 360;
                    else
                        thetas(:,j) = thetas(:,j) + 180;
                    end
                end
            angles{l}{j}{t} = thetas(:,j);
        end
        avgAngles{l}(t,:) = mean(thetas);
        minAngles{l}(t,:) = min(thetas);
        maxAngles{l}(t,:) = max(thetas);
    end
        for j=1:7
            angleSigns = sign(avgAngles{l}(:,j));
            if abs(sum(angleSigns)) < length(trialNames) %If some of the signs disagree with each other
                if sum(angleSigns) < 0 %If there are more negatives than positives
                    for t=1:length(trialNames)
                        if angleSigns(t) == 1 %If this trial has a positive average
                        angles{l}{j}{t} = angles{l}{j}{t} - abs(avgAngles{l}(t,j))*2; %Subtract 2x the average to shift if down
                        avgAngles{l}(t,j) = mean(angles{l}{j}{t});
                        minAngles{l}(t,j) = min(angles{l}{j}{t});
                        maxAngles{l}(t,j) = max(angles{l}{j}{t});
                        end
                    end
                else %If there are more positives than negatives
                    for t=1:length(trialNames)
                        if angleSigns(t) == -1 %If this trial has a negative average
                            angles{l}{j}{t} = angles{l}{j}{t} + abs(avgAngles{l}(t,j))*2; %Add 2x the average to shift if down
                            avgAngles{l}(t,j) = mean(angles{l}{j}{t});
                            minAngles{l}(t,j) = min(angles{l}{j}{t});
                            maxAngles{l}(t,j) = max(angles{l}{j}{t});
                        end
                    end
                end
            end
        end
    avgAngle(l,:) = mean(avgAngles{l});
    maxAngle(l,:) = mean(maxAngles{l});
    minAngle(l,:) = mean(minAngles{l});
    figure
    fig{l} = tiledlayout(3,3);
    fig{l}.TileSpacing = 'compact';
    fig{l}.Padding = 'compact';
    for j=1:7
        nexttile(j)
        for t=1:length(trialNames)
            plot(angles{l}{j}{t}')
            hold on
        end
        yline(avgAngle(l,j))
        yline(maxAngle(l,j),'LineStyle','--')
        yline(minAngle(l,j),'LineStyle','--')
        grid on
        title(jointNames{j})
        subtitle(['Avg: ' num2str(avgAngle(l,j)) ', Min: ' num2str(minAngle(l,j)) ', Max: ' num2str(maxAngle(l,j))])
    end
    title(fig{l},legNames{l})    
end