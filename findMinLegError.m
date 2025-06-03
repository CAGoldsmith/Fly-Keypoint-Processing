%Clarissa Goldsmith
%West Virginia University
%9/17/21
function [thetas,errors] = findMinLegError(leg,legInfo,frames,legName,filenameBase,savePath,savePathBase,segMags,CTr,TrF,FTi,TiTar,ThC1_start,ThC2_start,FTi_start,fixedJoints,toPlot)

if norm(fixedJoints) ~= 0
    fixedPos = importdata([savePath '\' filenameBase '_' legName '_Fixed Positions New.txt']);
else
    fixedPos = zeros(1,7);
end
%%%%%

%Body length of fly taken from Wosnitza et al. 2013 Table 1
bodyL = mean([2.06 2.12 2.09 2.07]); %in mm

% F(frames) = struct('cdata',[],'colormap',[]);
cond = [];
condShort = [];
if ~exist(savePath)
    if ~exist([savePathBase filenameBase])
        mkdir([savePathBase filenameBase])
    end
    mkdir(savePath)
end

error = [];
if toPlot
    fig = tiledlayout(3,4,'TileSpacing','tight','Padding','tight');
    [t,s] = title(fig, [filenameBase, ', ' legName],'Interpreter','none');
    t.FontSize = 16;
    frame_h = get(handle(gcf),'JavaFrame');
    set(frame_h,'Maximized',1);
end

errorMax = 1;
%Define the fixed positions for the joints based on the normal case
aEq = zeros(7,7);
if norm(fixedJoints) > 0
    for j=1:length(fixedJoints)
        aEq(j,j) = fixedJoints(j);
        if fixedJoints(j) == 1
            if isempty(cond)
                cond = [cond legInfo.joints{j} ' ' 'Fixed at ' num2str(fixedPos(j))];
                condShort = [condShort legInfo.joints{j}];
            else
                cond = [cond ', ' legInfo.joints{j} ' Fixed at ' num2str(fixedPos(j))];
                condShort = [condShort ', ' legInfo.joints{j}];

            end
        end
    end
    condShort = [condShort ' Fixed'];
else
    cond = 'Normal Case';
    condShort = 'Normal';
end


%For each frame:
for i=1:frames
    %keyboard
    posAnimal = [CTr(i,:);TrF(i,:);FTi(i,:);TiTar(i,:)];
    posEq = @(thetas) InvKinError(thetas,segMags(i,:),posAnimal);
    %options = optimoptions(@fminunc,'MaxFunctionEvaluations',7e10,'StepTolerance',1e-24);
    opts1 = optimset('MaxFunEvals',1e2000,'TolX',1e-12,'Display','off');
    opts2 = optimoptions('fminunc','OptimalityTolerance',1e-24,'Display','off','StepTolerance',1e-24);
    opts3 = optimoptions('fmincon','OptimalityTolerance',1e-24,'Display','off','StepTolerance',1e-24);
    if i == 1
        [thetas(i,:)] = fminsearch(posEq,[0,-ThC2_start,-ThC1_start,0,0,0,FTi_start],opts1);
        [thetas(i,:),error(i)] = fminunc(posEq,thetas(i,:),opts2);
    elseif i == 2
        [thetas(i,:),error(i)] = fminunc(posEq,thetas(i-1,:),opts2); %NORMAL CASE
        lb = thetas(i-1,:)-14;
        ub = thetas(i-1,:)+14;
        [thetas(i,:),error(i)] = fmincon(posEq,thetas(i,:),[],[],aEq,fixedPos,lb,ub,[],opts3);
    else
        lb = thetas(i-1,:)-14;
        ub = thetas(i-1,:)+14;
        thetasPred = 2*thetas(i-1,:) - thetas(i-2,:);
        [thetas(i,:),error(i)] = fmincon(posEq,thetasPred,[],[],aEq,fixedPos,lb,ub,[],opts3);
    end
    posRobot{i} = oneLegForwKin(thetas(i,:),segMags(i,:));
    [~,segNum] = size(segMags);

    for j=1:segNum
        errors(j,i) = norm((posRobot{i}(j,1:3) - posAnimal(j,1:3)) / segMags(i,j));
    end

    %Find vector directions for the strain gauges
    robotFemurVec = [posRobot{i}(3,1)-posRobot{i}(2,1), posRobot{i}(3,2)-posRobot{i}(2,2), posRobot{i}(3,3)-posRobot{i}(2,3)];
    robotTibiaVec = [posRobot{i}(4,1)-posRobot{i}(3,1), posRobot{i}(4,2)-posRobot{i}(3,2), posRobot{i}(4,3)-posRobot{i}(3,3)];

    animalFemurVec = [posAnimal(3,1)-posAnimal(2,1), posAnimal(3,2)-posAnimal(2,2), posAnimal(3,3)-posAnimal(2,3)];
    animalTibiaVec = [posAnimal(4,1)-posAnimal(3,1), posAnimal(4,2)-posAnimal(3,2), posAnimal(4,3)-posAnimal(3,3)];

    robotFeTinormVec(i,:) = cross(-robotFemurVec, robotTibiaVec);
    robotFeTinormVec(i,:) = robotFeTinormVec(i,:)/norm(robotFeTinormVec(i,:));

    animalFeTinormVec(i,:) = cross(-animalFemurVec, animalTibiaVec);
    animalFeTinormVec(i,:) = animalFeTinormVec(i,:)/norm(animalFeTinormVec(i,:));


    CSstartPt(i,:) = posRobot{i}(3,1:3) + .1*robotTibiaVec;
    robotGRFnormVec(i,:) = [robotFeTinormVec(i,1),robotFeTinormVec(i,2),0];
    robotGRFnormVec(i,:) = robotGRFnormVec(i,:)/norm(robotGRFnormVec(i,:));
    animalGRFnormVec(i,:) = [animalFeTinormVec(i,1),animalFeTinormVec(i,2),0];
    animalGRFnormVec(i,:) = animalGRFnormVec(i,:)/norm(animalGRFnormVec(i,:));
    robotFeTiVertAngle(i) = acosd(dot(robotFeTinormVec(i,:),robotGRFnormVec(i,:)));
    animalFeTiVertAngle(i) = acosd(dot(animalFeTinormVec(i,:),animalGRFnormVec(i,:)));

    if posRobot{i}(3,2) < posRobot{i}(4,2)
        robotFeTiVertAngle(i) = -robotFeTiVertAngle(i);
    end

    if posAnimal(3,2) < posAnimal(4,2)
        animalFeTiVertAngle(i) = -animalFeTiVertAngle(i);
    end

    FeTiVertAngleError(i) = animalFeTiVertAngle(i) - robotFeTiVertAngle(i);

    if toPlot

        %     Plot 3D comparison of the animal leg vs. the robot leg
        nexttile(1,[3,2])
        plot3([0;posAnimal(:,1)],[0;posAnimal(:,2)],[0;posAnimal(:,3)],'b-o')
        hold on
        plot3([0;posRobot{i}(:,1)],[0;posRobot{i}(:,2)],[0;posRobot{i}(:,3)],'r--o','LineWidth',1.5)
        legend('Animal','Robot','','Location','northeast');
        quiver3(CSstartPt(i,1),CSstartPt(i,2),CSstartPt(i,3),.05*robotFeTinormVec(i,1),.05*robotFeTinormVec(i,2),.05*robotFeTinormVec(i,3),'-g','AutoScaleFactor',3,'LineWidth',1.5,'MaxHeadSize',2,'Marker','s');
        xlabel('X')
        ylabel('Y')
        zlabel('Z')

        xMin = min(round(min(CTr(:,1))-.1,1),round(min(TiTar(:,1))-.1,1));
        if xMin > 0
            xMin = 0;
        end
        xMax = max(round(max(TiTar(:,1))+.1,1),round(max(CTr(:,1))+.1,1));
        yMin = min(round(min(CTr(:,2))-.1,1),round(min(TiTar(:,2))-.1,1));
        yMax = max(round(max(TiTar(:,2))+.1,1),round(max(CTr(:,2))+.1,1));
        zMin = round(min(TiTar(:,3))-.1,1);
        zMax = round(max(FTi(:,3))+.1,1);
        axis([xMin xMax yMin yMax zMin zMax])
        pbaspect([1 1 1])
        grid on
        hold off

        animalFootPos(i,:) = posAnimal(4,:);
        robotFootPos(i,:) = posRobot{i}(4,:);

        %Plot the X-Y projection of the legs
        nexttile(3)
        plot([0;posAnimal(:,1)],[0;posAnimal(:,2)],'b-o');
        hold on
        plot([0;posRobot{i}(:,1)],[0;posRobot{i}(:,2)],'r--o','LineWidth',1.5);
        quiver(CSstartPt(i,1),CSstartPt(i,2),.05*robotFeTinormVec(i,1),.05*robotFeTinormVec(i,2),'-g','AutoScaleFactor',3,'LineWidth',1.5,'MaxHeadSize',2,'Marker','s');
        title('X-Y Projection')
        xlabel('X')
        ylabel('Y')
        axis([xMin xMax yMin yMax]);
        grid on
        pbaspect([1 1 1])
        hold off

        %Plot the X-Z projection
        nexttile(4)
        plot([0;posAnimal(:,1)],[0;posAnimal(:,3)],'b-o');
        hold on
        plot([0;posRobot{i}(:,1)],[0;posRobot{i}(:,3)],'r--o','LineWidth',1.5);
        quiver(CSstartPt(i,1),CSstartPt(i,3),.05*robotFeTinormVec(i,1),.05*robotFeTinormVec(i,3),'-g','AutoScaleFactor',3,'LineWidth',1.5,'MaxHeadSize',2,'Marker','s');
        title('X-Z Projection')
        xlabel('X')
        ylabel('Z')
        axis([xMin xMax zMin zMax]);
        grid on
        pbaspect([1 1 1])
        hold off

        %Plot the Y-Z projection
        nexttile(7)
        plot([0;posAnimal(:,2)],[0;posAnimal(:,3)],'b-o');
        hold on
        plot([0;posRobot{i}(:,2)],[0;posRobot{i}(:,3)],'r--o','LineWidth',1.5);
        quiver(CSstartPt(i,2),CSstartPt(i,3),.05*robotFeTinormVec(i,2),.05*robotFeTinormVec(i,3),'-g','AutoScaleFactor',3,'LineWidth',1.5,'MaxHeadSize',2,'Marker','s');
        title('Y-Z Projection')
        xlabel('Y')
        ylabel('Z')
        axis([yMin yMax zMin zMax]);
        grid on
        pbaspect([1 1 1])
        hold off

        %     Plot how the TrF pitch relates to the TrF roll
        nexttile(8)
        plot(thetas(1:i,5),thetas(1:i,6))
        hold on
        plot(thetas(i,5),thetas(i,6),'o')
        xlabel('TrF Roll');
        ylabel('TrF Pitch');
        hold off

        nexttile(8)
        plot(animalFootPos(:,2),animalFootPos(:,3),'-o');
        hold on
        plot(robotFootPos(:,2),robotFootPos(:,3),'-o');
        axis([yMin yMax zMin zMax])
        grid on
        pbaspect([1 1 1])
        legend('Animal','Robot')
        hold off

        %     Plot how the errors develop over time for each joint and all combined
        nexttile(11)
        %     plot(error,'Color',[0 0.4470 0.7410],'LineWidth',1.5)
        %     hold on
        %     plot(i,error(i),'o','Color',[0 0.4470 0.7410])
        plot(errors(1,:),'Color',[0.8500 0.3250 0.0980])
        hold on
        plot(i,errors(1,i),'o','Color',[0.8500 0.3250 0.0980])
        plot(errors(2,:),'Color',[0.4940 0.1840 0.5560])
        plot(i,errors(2,i),'o','Color',[0.4940 0.1840 0.5560])
        plot(errors(3,:),'Color',[0.4660 0.6740 0.1880])
        plot(i,errors(3,i),'o','Color',[0.4660 0.6740 0.1880])
        plot(errors(4,:),'Color',[0.3010 0.7450 0.9330])
        plot(i,errors(4,i),'o','Color',[0.3010 0.7450 0.9330])
        legend('CTr','','TrF','','FTi','','TiTar','')
        title('Arc Length Error Over Time')
        ylabel('Error (in Number of Proximal Segments)');
        xlabel('Frame');
        grid on
        hold off
        if max(errors(:,i)) > errorMax && max(errors(:,i)) < errorMax + 10
            errorMax = max(errors(:,i));
        end
        axis([1 frames 0 errorMax])
        fixedJointsStr = ['[' num2str(fixedJoints) ']'];

        nexttile(12) %Plot how the angle of the Femur-Tibia plane from the vertical changes over time
        plot(1:i,robotFeTiVertAngle(1:i));
        hold on
        plot(i,robotFeTiVertAngle(i),'o');
        plot(1:i,animalFeTiVertAngle(1:i));
        plot(i,animalFeTiVertAngle(i),'o');
        axis([0 frames -90 90])
        title('Femur-tibia plane angle from X-Z plane');
        xlabel('Frame');
        ylabel('Angle (deg)');
        legend('Robot','','Animal','')
        grid on
        hold off

        %         nexttile(12) %Plot how the robot thetas change over time
        %         plot(1:i,thetas(1:i,1))
        %         hold on
        %         plot(1:i,thetas(1:i,2))
        %         plot(1:i,thetas(1:i,3))
        %         plot(1:i,thetas(1:i,4))
        %         plot(1:i,thetas(1:i,5),'LineWidth',1.5)
        %         plot(1:i,thetas(1:i,6),'LineWidth',1.5)
        %         plot(1:i,thetas(1:i,7))
        %         legend(legInfo.joints)
        %         title('Angle of Robot Joints Over Time');
        %         xlabel('Angle (deg)');
        %         axis([1 frames min(min(thetas)) max(max(thetas))])
        %         legend('','','','','TrF Roll','TrF Pitch','');
        %         hold off

        subtitle(fig,{cond, fixedJointsStr})
        %Convert the figure into a frame
        F(i) = getframe(gcf);
    end
end

if toPlot
    %Convert the frames into a gif
    gifName = [filenameBase '_' legName '_' 'Kinematic Matchup' '_' condShort '.gif'];

    for f=1:frames
        im{f} = frame2im(F(f));
        [A,map] = rgb2ind(im{f},256);
        if f == 1
            imwrite(A,map,[savePath '\' gifName],'gif','LoopCount',Inf,'DelayTime',1/10);
        else
            imwrite(A,map,[savePath '\' gifName],'gif','WriteMode','append','DelayTime',1/10);
        end
    end


    %Plot the final error plot on its own figure and save it
    fig2 = figure('WindowState','maximized');
    %plot(error,'Color',[0 0.4470 0.7410],'LineWidth',1.5)
    plot(errors(1,:),'Color',[0.8500 0.3250 0.0980])
    hold on
    plot(errors(2,:),'Color',[0.4940 0.1840 0.5560])
    plot(errors(3,:),'Color',[0.4660 0.6740 0.1880])
    plot(errors(4,:),'Color',[0.3010 0.7450 0.9330])
    legend('CTr','TrF','FTi','TiTar')
    axis([1 frames 0 errorMax])
    title([legName ' ' 'Error over Time'])
    ylabel('Number of Proximal Segments');
    subtitle(cond);
    saveas(fig2,[savePath '\' filenameBase '_' legName '_' 'Inv Kin Error Plot v2' '_' condShort '.png']);
    saveas(fig2,[savePath '\' filenameBase '_' legName '_' 'Inv Kin Error Plot v2' '_' condShort '.fig']);

    %Plot the final plane angle plot on its own figure and save it
    fig3 = figure('WindowState','maximized');
    %plot(error,'Color',[0 0.4470 0.7410],'LineWidth',1.5)
    plot(robotFeTiVertAngle(1,:))
    hold on
    plot(animalFeTiVertAngle(1,:))
    axis([1 frames -90 90])
    title([legName ' ' 'FeTi leg plane angle from vertical over time'])
    ylabel('Angle (deg)');
    legend('Robot','Animal')
    subtitle(cond);
    saveas(fig3,[savePath '\' filenameBase '_' legName '_' 'FeTi Plane Angle Plot' '_' condShort '.png']);
    saveas(fig3,[savePath '\' filenameBase '_' legName '_' 'FeTi Plane Angle Plot' '_' condShort '.fig']);

    fig4 = figure('WindowState','maximized');
    plot(animalFootPos(:,2),animalFootPos(:,3),'-o');
    hold on
    plot(robotFootPos(:,2),robotFootPos(:,3),'-o');
    axis([yMin yMax zMin zMax])
    grid on
    title([legName ' ' 'TiTar Position'])
    xlabel('Y Axis')
    ylabel('Z Axis')
    legend('Animal','Robot');
    saveas(fig4,[savePath '\' filenameBase '_' legName '_' 'TiTar Positions' '_' condShort '.png']);
    saveas(fig4,[savePath '\' filenameBase '_' legName '_' 'TiTar Positions' '_' condShort '.fig']);
end

if norm(fixedJoints) == 0
    fixedPos = mean(thetas);
    fixedPos = round(fixedPos);
    save([savePath '\' filenameBase '_' legName '_Fixed Positions New.txt'],'fixedPos','-ASCII');
end

avgPosError = mean(error);
avgLegPlaneError = mean(FeTiVertAngleError);
legPlaneRange = max(robotFeTiVertAngle) - min(robotFeTiVertAngle);

save([savePath '\' filenameBase '_' legName '_Analysis Vars_' condShort '.mat'],'errors','avgPosError', 'avgLegPlaneError','legPlaneRange');

%Store the theta values as a global variable and in a text file
thetaFilename = [savePath '\' filenameBase '_' legName '_Robot Angles New_' condShort];
save([thetaFilename '.txt'],'thetas','-ASCII');
save([thetaFilename '.mat'],'thetas');

end
