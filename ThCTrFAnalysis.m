%Clarissa Goldsmith
%West Virginia University
%Coding began: 9/7/21
close all
clear all
%Load in the keypoint file
filepath = 'C:\Users\Clarissa G\Videos\201218_bolt-Chr\201218_bolt-Chr_Fly06_M_T011';
filename = '201218_bolt-Chr_Fly06_M_T011_keypoints';
filenameBase = '201218_bolt-Chr_Fly06_M_T011';
file = [filepath '\' filename '.mat'];
load(file)
if not(isfolder(filenameBase))
    mkdir(filenameBase)
end

savePathBase = 'C:\Users\Clarissa G\Documents\MATLAB\Fly Keypoint Processing\';

[frames, joints, dims] = size(keypoints3D); %Determine the size of the file

keypoints3D(:,:,3) = -keypoints3D(:,:,3); %Flip the z axis so it's right side up
keypoints3D(:,:,1) = -keypoints3D(:,:,1);

%Make everything a bit larger
keypoints3D(:,:,:) = keypoints3D(:,:,:)*4;

%In file:   1:6 is RF leg, lowID:highID is RM, 13:1lowID+1 is RH, 1lowID+2:24 is LF, 25:30 is
%LM, 31:36 is LH
%Order of points is: ThC, CTr, TrF, FTi, TiTar, Tar
legInfo.lowIDs = [1 7 13 19 25 31];
legInfo.highIDs = [6 12 18 24 30 36];
legInfo.leg = {'Right Front'; 'Right Middle'; 'Right Hind'; 'Left Front'; 'Left Middle'; 'Left Hind'};
legInfo.az = [45,-45];
legInfo.el = 40;

%DEFINE THE LEG NUMBER HERE:
leg = 2;

lowID = legInfo.lowIDs(leg);
hiID = legInfo.highIDs(leg);
legName = legInfo.leg{leg};

savePath = [savePathBase filenameBase '\' legName];

if not(isfolder(savePath))
    mkdir(savePath)
end

%Determine the min values for each dimension
Xmin = min(nonzeros(min(keypoints3D(:,:,1))));
Ymin = min(nonzeros(min(keypoints3D(:,:,2))));
Zmin = min(nonzeros(min(keypoints3D(:,:,3))));
%Normalize the data based on the min values
keypoints3D(:,:,1) = keypoints3D(:,:,1) - Xmin;
keypoints3D(:,:,2) = keypoints3D(:,:,2) - Ymin;
keypoints3D(:,:,3) = keypoints3D(:,:,3) - Zmin;
%Find the max values of each dimension after normalizing
if leg/6 <= 0.5
    Xmax = max(max(keypoints3D(:,1:18,1)));
    Ymax = max(max(keypoints3D(:,1:18,2)));
    Zmax = max(max(keypoints3D(:,1:18,3)));
    %if max(max(keypoints3D(:,37:41,1)))
    Xmin = 2;
else
    Xmax = max(max(keypoints3D(:,19:36,1)));
    Ymax = max(max(keypoints3D(:,19:36,2)));
    Zmax = max(max(keypoints3D(:,19:36,3)));
    Xmin = -4;
end

angleAxesMax = [40 40 90 90];
angleAxesMin = [-40 -40 0 0];

%Generate an empty frame struct, an empty figure at fullscreen, and a bunch
%of empty matrices to fill
F(frames) = struct('cdata',[],'colormap',[]);
fig = tiledlayout(4,4,'TileSpacing','Tight');
title(fig, legName);
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);

ThCTotAngle = zeros(frames,1);
angleBtwnTrFPlanes = zeros(frames,1);
CoxTrocNormVec = zeros(frames,3);
FeTiNormVec = zeros(frames,3);
TrocFemAngle = zeros(frames,1);
ThC1Angle = zeros(frames,1);
ThC2Angle = zeros(frames,1);
TrocFemPlanarAngle = zeros(frames,1);

%Generate a filename for the eventual GIF
gifName = [filenameBase '_' legName '_' 'Full' '.gif'];

%Assume that the plane bisecting the thorax is going to be parallel to the y-z plane
%To minimize the angle between the planes, will place "normal" vector for
%thorax along y axis
ThoraxNormVec = [0,1,0];

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

for i=1:frames %For each frame:
    %In the first tile, plot the chosen leg with the defined view
    nexttile(1,[4,2])
    plot3(keypoints3D(i,lowID:hiID,1),keypoints3D(i,lowID:hiID,2),keypoints3D(i,lowID:hiID,3),'o','LineWidth',2);
    %keyboard
    hold on
    for k=lowID:hiID-1
        plot3(keypoints3D(i,k:k+1,1),keypoints3D(i,k:k+1,2),keypoints3D(i,k:k+1,3),'LineWidth',2);
    end
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on
    if leg/6 <= 0.5
        view(legInfo.az(1),legInfo.el);
    else
        view(legInfo.az(2),legInfo.el);
    end
    %Also plot the thorax plane and various parts of the thorax, to make
    %sure we're oriented correctly
    fill3(ThC(i,1)*ones(1,4), [keypoints3D(i,37,2) keypoints3D(i,37,2) keypoints3D(i,40,2) keypoints3D(i,40,2)], [ThC(i,3), keypoints3D(i,37,3), keypoints3D(i,37,3), ThC(i,3)], 'k','FaceAlpha',0.5);
    plot3(keypoints3D(i,37:41,1),keypoints3D(i,37:41,2),keypoints3D(i,37:41,3),'*');
    %keyboard
    
    %Find the normal unit vector for the plane going through the coxa and the
    %trochanter (given that the CTr joint is a pure hinge)
    CoxTrocNormVec(i,:) = cross(coxaVec(i,:), trocVec(i,:));
    CoxTrocNormVec(i,:) = CoxTrocNormVec(i,:)/norm(CoxTrocNormVec(i,:));
    if CoxTrocNormVec(i,2) < 0
        CoxTrocNormVec(i,:) = -CoxTrocNormVec(i,:);
    end
    
    %Find the normal unit vector for the plane going through the femur and
    %tibia
    FeTiNormVec(i,:) = cross(femurVec(i,:), tibiaVec(i,:));
    FeTiNormVec(i,:) = FeTiNormVec(i,:)/norm(FeTiNormVec(i,:));
    
    %Calculate the angle between the Coxa-Troc normal vector and the thorax 
    %normal vector and convert it to degrees
    ThCTotAngle(i) = acos(dot(CoxTrocNormVec(i,:),ThoraxNormVec));
    ThCTotAngle(i) = ThCTotAngle(i)*180/pi;
    
    %Calculate the angle between the Femur-Tibia normal vector and the Coxa-Troc 
    %normal vector and convert it to degrees
    angleBtwnTrFPlanes(i) = acos(dot(CoxTrocNormVec(i,:),FeTiNormVec(i,:)));
    angleBtwnTrFPlanes(i) = angleBtwnTrFPlanes(i)*180/pi;
    
    %Calculate the angle of the x-z projection of the coxa from the
    %vertical (a.k.a ThC 2)
    ThC2Angle(i) =  atan(coxaVec(i,1)/coxaVec(i,3));
    ThC2Angle(i) = ThC2Angle(i)*180/pi;

    %Use the ThC 2 angle to find the ThC 1 angle
    %Align the axes with the coxa by rotating around the y axis
    omegaHatCoxa = [0 0 1; 0 0 0; -1 0 0];
    Rx = eye(3) + omegaHatCoxa*sind(-ThC2Angle(i)) + omegaHatCoxa^2*(1-cosd(-ThC2Angle(i)));
    coxaThC1Plane(i,:) = (Rx*coxaVec(i,:)')';
    ThC1Angle(i) = atand(coxaThC1Plane(i,2)/coxaThC1Plane(i,3));
    
    %Calculate the angle between the Troc and the Femur on a Tro-Fe plane
    TrocFemPlanarAngle(i) = acosd(dot(-trocVec(i,:),femurVec(i,:))/(norm(femurVec(i,:))*norm(trocVec(i,:)))); %#ok<*SAGROW>
    
    %Add the normal vectors to the plot starting from the joint of interest
    plot3([ThC(i,1) ThC(i,1)+CoxTrocNormVec(i,1)],[ThC(i,2) ThC(i,2)+CoxTrocNormVec(i,2)],[ThC(i,3) ThC(i,3)+CoxTrocNormVec(i,3)],'--b');
    plot3([ThC(i,1) ThC(i,1)+ThoraxNormVec(1)],[ThC(i,2) ThC(i,2)+ThoraxNormVec(2)],[ThC(i,3) ThC(i,3)+ThoraxNormVec(3)],'--k');
    
    plot3([TrF(i,1) TrF(i,1)+CoxTrocNormVec(i,1)],[TrF(i,2) TrF(i,2)+CoxTrocNormVec(i,2)],[TrF(i,3) TrF(i,3)+CoxTrocNormVec(i,3)],'--b');
    plot3([TrF(i,1) TrF(i,1)+FeTiNormVec(i,1)],[TrF(i,2) TrF(i,2)+FeTiNormVec(i,2)],[TrF(i,3) TrF(i,3)+FeTiNormVec(i,3)],'--r');

    %Find the points for the coxaPlane and femurPlane for plotting
    coxaPlane = [ThC(i,1)+coxaVec(i,1) ThC(i,2)+coxaVec(i,2) ThC(i,3)+coxaVec(i,3);...
            CTr(i,1)-coxaVec(i,1) CTr(i,2)-coxaVec(i,2) CTr(i,3)-coxaVec(i,3);...
            (CTr(i,1)-coxaVec(i,1))+trocVec(i,1)*2 CTr(i,2)-coxaVec(i,2)+trocVec(i,2)*2 CTr(i,3)-coxaVec(i,3)+trocVec(i,3)*2;...
            ThC(i,1)+coxaVec(i,1)+trocVec(i,1)*2 ThC(i,2)+coxaVec(i,2)+trocVec(i,2)*2 ThC(i,3)+coxaVec(i,3)+trocVec(i,3)*2];
        
    femurPlane = [TrF(i,1)-tibiaVec(i,1)/2 TrF(i,2)-tibiaVec(i,2)/2 TrF(i,3)-tibiaVec(i,3)/2;...
        FTi(i,1)-tibiaVec(i,1)/2 FTi(i,2)-tibiaVec(i,2)/2 FTi(i,3)-tibiaVec(i,3)/2;...
        TiTar(i,1) TiTar(i,2) TiTar(i,3);...
        TrF(i,1)+tibiaVec(i,1)/2 TrF(i,2)+tibiaVec(i,2)/2 TrF(i,3)+tibiaVec(i,3)/2];
    
    %Plot the planes on the figure
    fill3(coxaPlane(:,1), coxaPlane(:,2), coxaPlane(:,3), 'b','FaceAlpha',0.5);
    fill3(femurPlane(:,1), femurPlane(:,2), femurPlane(:,3), 'r','FaceAlpha',0.5);

    hold off
    %Change the axes, add a title and a legend
    %axis([Xmin Xmax 0 Ymax 0 Zmax]);
    axis([0 16 0 16 0 16]);
    lgd = legend('Joints','Coxa','Trochanter','Femur','Tibia','Tarsus','Thorax Plane','Body Points','Coxa-Troc Plane Normal Vector','Thorax Plane "Normal" Vector','','Femur-Tibia Plane Normal Vector','Coxa-Trochanter Plane','Femur-Tibia Plane');
    lgd.Location = 'northeast';
    %In the next tile, plot the projection of the coxa on the x-z plane
    nexttile(3)
    
    plot(0,0,'ko');
    hold on
    plot([0 coxaVec(i,1)],[0 coxaVec(i,3)])
    plot(coxaVec(1:i,1), coxaVec(1:i,3),'o');
    axis square
    axis([-1 1 -2 0]);
    xlabel('X');
    ylabel('Z');
    grid on
    title('Projection of Coxa on X-Z Plane')
    hold off
    
    %In the next tile, plot the projection of the coxa on the y-z plane
    nexttile(4)
    plot(0,0,'ko');
    hold on 
    plot([0 coxaThC1Plane(i,2)],[0 coxaThC1Plane(i,3)])
    plot(coxaThC1Plane(1:i,2), coxaThC1Plane(1:i,3),'o');
    axis square
    axis([-1 1 -2 0]);
    xlabel('Y');
    ylabel('Z*');
    grid on
    title('Plane Containing ThC and CTr and Parallel to Y Axis')
    hold off
    
    %Plot the ThC component angles over time under the corresponding projection    
    nexttile(7)
    plot(ThC2Angle(1:i));
    hold on
    plot(i,ThC2Angle(i),'o');
    title('Angle of ThC2');
    ylabel('Angle (deg)');
    if ThC2Angle(i) > angleAxesMax(1)
        angleAxesMax(1) = ThC2Angle(i);
    end
    if ThC2Angle(i) < angleAxesMin(1)
        angleAxesMin(1) = ThC2Angle(i);
    end
    axis([0 frames angleAxesMin(1) angleAxesMax(1)])
    grid on
    hold off
    
    nexttile(8)
    plot(ThC1Angle(1:i));
    hold on
    plot(i,ThC1Angle(i),'o');
    title('Angle of ThC1');
    ylabel('Angle (deg)');
    
    if ThC1Angle(i) > angleAxesMax(2)
        angleAxesMax(2) = ThC1Angle(i);
    end
    if ThC1Angle(i) < angleAxesMin(2)
        angleAxesMin(2) = ThC1Angle(i);
    end
    axis([0 frames angleAxesMin(2) angleAxesMax(2)])
    grid on
    hold off
    


    %Plot how the angle between the Thorax and Coxa-Troc planes changes
    %over time
    nexttile(11)
    plot(ThCTotAngle(1:i))
    hold on
    plot(i,ThCTotAngle(i),'o');
    title('Angle Between Thorax Plane and Coxa-Troc Plane')
    xlabel('Frame')
    ylabel('Angle (deg)')
    if ThCTotAngle(i) > angleAxesMax(3)
        angleAxesMax(3) = ThCTotAngle(i);
    end
    if ThCTotAngle(i) < angleAxesMin(3)
        angleAxesMin(3) = ThCTotAngle(i);
    end
    axis([0 frames angleAxesMin(3) angleAxesMax(3)])
    grid on
    hold off
    
    %Plot how the angle between the Coxa-Troc and Femur-Tibia planes
    %changes over time
    nexttile(12)
    plot(angleBtwnTrFPlanes(1:i))
    hold on
    plot(i,angleBtwnTrFPlanes(i),'o');
    title('Angle Between Coxa-Troc Plane and Femur-Tibia Plane')
    ylabel('Angle (deg)')
    if angleBtwnTrFPlanes(i) > angleAxesMax(4)
        angleAxesMax(4) = angleBtwnTrFPlanes(i);
    end
    if angleBtwnTrFPlanes(i) < angleAxesMin(4)
        angleAxesMin(4) = angleBtwnTrFPlanes(i);
    end
    axis([0 frames angleAxesMin(4) angleAxesMax(4)]);
    grid on
    hold off
    
    nexttile(15)
    plot(TrocFemPlanarAngle(1:i), angleBtwnTrFPlanes(1:i))
    hold on
    xlabel('TrF Pitch')
    ylabel('TrF Yaw')
    plot(TrocFemPlanarAngle(i),angleBtwnTrFPlanes(i),'o')
    axis([120 180 0 60]);
    grid on
    hold off
    
    nexttile(16)
    plot(TrocFemPlanarAngle(1:i));
    hold on
    plot(i,TrocFemPlanarAngle(i),'o');
    title('Angle Between Trochanter and Femur In-Plane');
    xlabel('Frame');
    axis([0 frames 90 180]);
    grid on
    hold off
 
    %Store the figure as a frame
    F(i) = getframe(gcf);
end
%Convert the frames into a GIF
for j=1:frames
    im{j} = frame2im(F(j));
    [A,map] = rgb2ind(im{j},256);
    if j == 1
        imwrite(A,map,[savePath '\' gifName],'gif','LoopCount',Inf,'DelayTime',1/10);
    else
        imwrite(A,map,[savePath '\' gifName],'gif','WriteMode','append','DelayTime',1/10);
    end
end

%Make larger figures of each angle plot
fig2 = figure('WindowState','maximized');
plot(ThC2Angle,'LineWidth',2);
set(gca,'FontSize',16);
title('Angle of ThC2','FontSize',18);
ylabel('Angle (deg)','FontSize',16);
xlabel('Frame','FontSize',16)
axis([0 frames floor(min(ThC2Angle)/10)*10 ceil(max(ThC2Angle)/10)*10])
grid on
saveas(fig2,[savePath '\' filenameBase '_' legName '_' 'ThC2Angle' '.png']);
saveas(fig2,[savePath '\' filenameBase '_' legName '_' 'ThC2Angle' '.fig']);

fig3 = figure('WindowState','maximized');
set(gca,'FontSize',16);
plot(ThC1Angle,'LineWidth',2);
title('Angle of ThC1','FontSize',18);
ylabel('Angle (deg)','FontSize',16);
xlabel('Frame','FontSize',16)
axis([0 frames  floor(min(ThC1Angle)/10)*10 ceil(max(ThC1Angle)/10)*10]);
grid on
saveas(fig3,[savePath '\' filenameBase '_' legName '_' 'ThC1Angle' '.png']);
saveas(fig3,[savePath '\' filenameBase '_' legName '_' 'ThC1Angle' '.fig']);

fig4 = figure('WindowState','maximized');
set(gca,'FontSize',16);
plot(ThCTotAngle,'LineWidth',2)
title('Angle Between Thorax Plane and Coxa-Troc Plane','FontSize',18)
xlabel('Frame','FontSize',16)
ylabel('Angle (deg)','FontSize',16)
axis([0 frames  floor(min(ThCTotAngle)/10)*10 ceil(max(ThCTotAngle)/10)*10]);
grid on
saveas(fig4,[savePath '\' filenameBase '_' legName '_' 'ThCTotAngle' '.png']);
saveas(fig4,[savePath '\' filenameBase '_' legName '_' 'ThCTotAngle' '.fig']);

fig5 = figure('WindowState','maximized');
plot(angleBtwnTrFPlanes,'LineWidth',2)
set(gca,'FontSize',16);
title('Angle Between Coxa-Troc Plane and Femur-Tibia Plane','FontSize',18)
ylabel('Angle (deg)','FontSize',16)
xlabel('Frame','FontSize',16)
axis([0 frames  floor(min(angleBtwnTrFPlanes)/10)*10 ceil(max(angleBtwnTrFPlanes)/10)*10])
grid on
saveas(fig5,[savePath '\' filenameBase '_' legName '_' 'angleBtwnTrFPlanes' '.png']);
saveas(fig5,[savePath '\' filenameBase '_' legName '_' 'angleBtwnTrFPlanes' '.fig']);

fig6 = figure('WindowState','maximized');
set(gca,'FontSize',16);
plot(TrocFemPlanarAngle,'LineWidth',2);
title('Angle Between Trochanter and Femur In-Plane','FontSize',18);
xlabel('Frame','FontSize',16);
ylabel('Angle(deg)','FontSize',16);
axis([0 frames floor(min(TrocFemPlanarAngle)/10)*10 ceil(max(TrocFemPlanarAngle)/10)*10]);
grid on
set(gca,'FontSize',16);
saveas(fig6,[savePath '\' filenameBase '_' legName '_' 'TrocFemPlanarAngle' '.png']);
saveas(fig6,[savePath '\' filenameBase '_' legName '_' 'TrocFemPlanarAngle' '.fig']);
