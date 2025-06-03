function pos = oneLegForwKin(thetas,mags)
%Assumes that thetas are given in the order: [ThC1 ThC2 ThC3 CTr TrF1 TrF2 FTi]
%Assumes mags is organized: [coxaMag, trocMag, femurMag, tibiaMag];
%Frames are labeled as follows:
%FTi = D  TrF1 (roll) = C   TrF2 (pitch) = C1   CTr = B    ThC1 = A1   ThC2 = A2   ThC3 = A3
%%
%Baseline equations:
exp_w = @(theta,wHat) eye(3)+wHat*sind(theta)+wHat^2*(1-cosd(theta));
g = @(theta,wHat,v) [exp_w(theta,wHat), (eye(3)-exp_w(theta,wHat))*wHat*v; 0 0 0 1];

%Zero positions for each joint
TiTar_zero = [mags(2)+mags(3);0;-mags(1)-mags(4);1];
FTi_zero = [mags(2)+mags(3);0;-mags(1);1];
TrF_zero = [mags(2);0;-mags(1);1];
CTr_zero = [0;0;-mags(1);1];
ThC_zero = [0;0;0;1];

%Find g matrix for rotation of D frame (FTi joint rotation)
w.DA = [0;1;0];
wHat.DA = hat(w.DA);
v.DA = -wHat.DA*FTi_zero(1:3);
g_DA = g(thetas(7),wHat.DA,v.DA);

%Find the g matrix for pitch of C frame (TrF pitch)

w.C1A = [0;1;0];
wHat.C1A = hat(w.C1A);
v.C1A = -wHat.C1A*TrF_zero(1:3);
g_C1A = g(thetas(6),wHat.C1A,v.C1A);

%Find the g matrix for rotation of the C frame(TrF roll)
w.C2A = [1;0;0];
wHat.C2A = hat(w.C2A(1:3));
v.C2A = -wHat.C2A*TrF_zero(1:3);
g_C2A = g(thetas(5),wHat.C2A,v.C2A);

%Find the g matrix for rotation of the B frame
w.BA = [0;1;0];
wHat.BA = hat(w.BA);
v.BA = -wHat.BA*CTr_zero(1:3);
g_BA = g(thetas(4),wHat.BA,v.BA);

%Find g matrix for a "back-and-forth" pitch of the A frame (ThC1)
w.A1 = [1;0;0];
wHat.A1 = hat(w.A1);
v.A1 = -wHat.A1*ThC_zero(1:3);
g_A1 = g(thetas(1),wHat.A1,v.A1);

%Find g matrix for a "up and down" yaw of the A frame (ThC2)
w.A2 = [0;1;0];
wHat.A2 = hat(w.A2(1:3));
v.A2 = -wHat.A2*ThC_zero(1:3);
g_A2 = g(thetas(2),wHat.A2,v.A2);

%Find g matrix for a roll of the A frame (ThC3)
w.A3 = [0;0;1];
wHat.A3 = hat(w.A3);
v.A3 = -wHat.A3*ThC_zero(1:3);
g_A3 = g(thetas(3),wHat.A3,v.A3);

%Multiply the zero points by the matrices
TiTar_pos = g_A2*g_A3*g_A1*g_BA*g_C1A*g_C2A*g_DA*TiTar_zero;
FTi_pos = g_A2*g_A3*g_A1*g_BA*g_C1A*g_C2A*FTi_zero;
TrF_pos = g_A2*g_A3*g_A1*g_BA*TrF_zero;
CTr_pos = g_A2*g_A3*g_A1*CTr_zero;

pos = [CTr_pos'; TrF_pos'; FTi_pos'; TiTar_pos'];

end