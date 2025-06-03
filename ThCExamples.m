%Clarissa Goldsmith
%West Virginia University
%Coding began: 9/7/21
%Create idealized motion of ThC joint with 1-3 degrees of freedom
step = [2,0,5];
frames = 50;
%Pick some multiple of the step
thc1_range = 40;
thc2_range = 15;
thc3_range = 20;

%Equilibrium of front leg coxa assumed to be 30 deg from horizontal and 45 deg from vertical
%Establish the starting position
figure
fig = tiledlayout(2,2);
nexttile(1,[2,1])
coxaVec = zeros(frames,3);
coxaNorm = zeros(frames,3);
angle = zeros(frames,1);
coxaVec_init = [cosd(30); sind(30); 0];
R = [cosd(45) 0 sind(45); 0 1 0; -sind(45) 0 cosd(45)];
coxaVec_init = R*coxaVec_init;
omega = coxaVec_init;
coxaNorm_init = [0; 1; 0];
plot3([0 coxaVec_init(1)],[0 coxaVec_init(2)], [0 coxaVec_init(3)],'k-o')
hold on
plot3([0 coxaNorm_init(1)],[0 coxaNorm_init(2)], [0 coxaNorm_init(3)],'k--')
view(60,50)
xlabel('X')
ylabel('Y')
zlabel('Z')
axis([0 1 0 1 -1 0])
grid on

theta1 = 0;
theta2 = -15;
theta3 = 0;
dir = [1,1,1]; %1 for forward, -1 for backward
for i=1:frames
    if thc1_range ~= 0 %If we've specified ThC 1 rotation
        %If we've reached the end of the range in either direction, switch
        %direction
        if abs(theta1) == thc1_range
             if theta1 == thc1_range
                 dir(1) = -1;
             else
                 dir(1) = 1;
             end
        end
        %Move a step in the specified direction
        theta1 = theta1 + step(1)*dir(1);
        %Then calculate the R matrix for a rotation of theta around the z axis
        R_theta1 = [cosd(theta1) sind(theta1) 0; -sind(theta1) cosd(theta1) 0; 0 0 1];
    else
        R_theta1 = [1 0 0; 0 1 0; 0 0 1];
    end
    
    if thc2_range ~= 0 %If we've specified ThC 2 rotation
        %If we've reached the end of the range in either direction, switch
        %direction
        if abs(theta2) == thc2_range
             if theta2 == thc2_range
                 dir(2) = -1;
             else
                 dir(2) = 1;
             end
        end
        %Move a step in the specified direction
        theta2 = theta2 + step(2)*dir(2);
        %Then calculate the R matrix for a rotation of theta around the y axis
        R_theta2 = [cosd(theta2) 0 -sind(theta2); 0 1 0; sind(theta2) 0 cosd(theta2)];
    else
        R_theta2 = [1 0 0; 0 1 0; 0 0 1];
    end
    
    if thc3_range ~= 0 %If we've specified ThC 3 rotation
        %This is where we have to utilize exponents
        %If we've reached the end of the range in either direction, switch
        %direction
        if abs(theta3) == thc3_range
             if theta3 == thc3_range
                 dir(3) = -1;
             else
                 dir(3) = 1;
             end
        end
        %Move a step in the specified direction
        theta3 = theta3 + step(3)*dir(3);
        %Find the omega hat matrix from the omega vector
        omega_hat = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
        R_theta3 = eye(3)+ omega_hat*sind(theta3) + omega_hat^2*(1-cosd(theta3));
        
    else
        R_theta3 = [1 0 0; 0 1 0; 0 0 1];
    end
    
    %Multiply to find the new position of the coxa and the normal vector
    coxaVec(i,:) = R_theta2*R_theta1*coxaVec_init;
    coxaNorm(i,:) = R_theta3*R_theta2*R_theta1*coxaNorm_init;
    
    %Find the angle between the current normal vector and the initial
    angle(i) = acos(dot(coxaNorm(i,:),coxaNorm_init));
    angle(i) = angle(i)*180/pi;
    
    %Plot the new positions
    color = [rand,rand,rand];
    nexttile(1)
    plot3([0 coxaVec(i,1)],[0 coxaVec(i,2)], [0 coxaVec(i,3)],'-o','Color',color)
    plot3([0 coxaNorm(i,1)],[0 coxaNorm(i,2)], [0 coxaNorm(i,3)],'--','Color',color);
    
    %Plot the angle change over time
    nexttile(2)
    plot(angle);
    hold on
    plot(angle(i),'o')
    hold off
    axis([0 frames 0 90]);
    title('Angle Between Normal Vectors')
    
    nexttile(4)
    plot(0,0,'ko');
    hold on
    plot(coxaNorm(1:i,1), coxaNorm(1:i,3));
    plot(coxaNorm(i,1), coxaNorm(i,3));
    hold off
    legend('Tip of Body Normal Vector','Tip of Coxa Normal Vector');
    xlabel('X')
    ylabel('Z')
    %keyboard
end
    
