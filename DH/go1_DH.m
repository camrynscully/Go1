%% Kinematic Model of the Go1
% "floating table" model of the Go1 


clear all
close all

gray = [.7 .7 .7];

% data = load('standing_joint_q.txt');
data = load('/Users/camrynscully/Desktop/go1/Data/walking_joint_q.txt');

% length of each link [m]
bw = 0.0935;
bl = 0.3762;
l0 = 0.08;
l1 = 0.213;
l2 = 0.213;

zB = 0;
pB = [0; 0; zB];

%% Front Right Leg
alphaFR = [pi/2 0 0 -pi/2 0 0];
thetaFR = [pi/2 0 0 -pi/2 0 0];
dFR = [0 bl/2 0 0 0 0];
aFR = [-bw/2 0 -l0 0 l1 l2];
FR = 57;

% Rotation from Origin (A) to Base (B)
RFR_AB = dh_matrix(dFR(1),thetaFR(1),aFR(1),alphaFR(1));

% Rotation from Base to 0 : Hip A/A
RFR_B0 = dh_matrix(dFR(2),thetaFR(2),aFR(2),alphaFR(2));
RFR_A0 = RFR_AB*RFR_B0;

% Rotation from 0 to 1 
RFR_01 = dh_matrix(dFR(3),thetaFR(3)+data(FR,1),aFR(3),alphaFR(3));
RFR_A1 = RFR_AB*RFR_B0*RFR_01;

% Rotation from 1 to 2 : Hip F/E
RFR_12 = dh_matrix(dFR(4),thetaFR(4),aFR(4),alphaFR(4));
RFR_A2 = RFR_AB*RFR_B0*RFR_01*RFR_12;

% Rotation from 2 to 3 : Knee 
RFR_23 = dh_matrix(dFR(5),thetaFR(5)+data(FR,2),aFR(5),alphaFR(5));
RFR_A3 = RFR_AB*RFR_B0*RFR_01*RFR_12*RFR_23;

% Rotation from 3 to End-Effector
RFR_3E = dh_matrix(dFR(6),thetaFR(6)+data(FR,3),aFR(6),alphaFR(6));
RFR_AE = RFR_AB*RFR_B0*RFR_01*RFR_12*RFR_23*RFR_3E;

%% Front Left Leg
alphaFL = [pi/2 0 0 -pi/2 0 0];
thetaFL = [pi/2 0 0 -pi/2 0 0];
dFL = [0 bl/2 0 0 0 0];
aFL = [bw/2 0 l0 0 l1 l2];
FL = 58;

% Rotation from Origin (A) to Base (B)
RFL_AB = dh_matrix(dFL(1),thetaFL(1),aFL(1),alphaFL(1));

% Rotation from B to 0 : Hip A/A
RFL_B0 = dh_matrix(dFL(2),thetaFL(2),aFL(2),alphaFL(2));
RFL_A0 = RFL_AB*RFL_B0;

% Rotation from 0 to 1 
RFL_01 = dh_matrix(dFL(3),thetaFL(3)+data(FL,1),aFL(3),alphaFL(3));
RFL_A1 = RFL_AB*RFL_B0*RFL_01;

% Rotation from 1 to 2 : Hip F/E
RFL_12 = dh_matrix(dFL(4),thetaFL(4),aFL(4),alphaFL(4));
RFL_A2 = RFL_AB*RFL_B0*RFL_01*RFL_12;

% Rotation from 2 to 3 : Knee
RFL_23 = dh_matrix(dFL(5),thetaFL(5)+data(FL,2),aFL(5),alphaFL(5));
RFL_A3 = RFL_AB*RFL_B0*RFL_01*RFL_12*RFL_23;

% Rotation from 3 to End-Effector
RFL_3E = dh_matrix(dFL(6),thetaFL(6)+data(FL,3),aFL(6),alphaFL(6));
RFL_AE = RFL_AB*RFL_B0*RFL_01*RFL_12*RFL_23*RFL_3E;

%% Rear Right Leg
alphaRR = [pi/2 0 0 -pi/2 0 0];
thetaRR = [pi/2 0 0 -pi/2 0 0];
dRR = [0 -bl/2 0 0 0 0];
aRR = [-bw/2 0 -l0 0 l1 l2];
RR = 59;

% Rotation from Origin (A) to Base (B) 
RRR_AB = dh_matrix(dRR(1),thetaRR(1),aRR(1),alphaRR(1));

% Rotation from B to 0 : Hip A/A
RRR_B0 = dh_matrix(dRR(2),thetaRR(2),aRR(2),alphaRR(2));
RRR_A0 = RRR_AB*RRR_B0;

% Rotation from 0 to 1 
RRR_01 = dh_matrix(dRR(3),thetaRR(3)+data(RR,1),aRR(3),alphaRR(3));
RRR_A1 = RRR_AB*RRR_B0*RRR_01;

% Rotation from 1 to 2 : Hip F/E
RRR_12 = dh_matrix(dRR(4),thetaRR(4),aRR(4),alphaRR(4));
RRR_A2 = RRR_AB*RRR_B0*RRR_01*RRR_12;

% Rotation from 2 to 3 : Knee
RRR_23 = dh_matrix(dRR(5),thetaRR(5)+data(RR,2),aRR(5),alphaRR(5));
RRR_A3 = RRR_AB*RRR_B0*RRR_01*RRR_12*RRR_23;

% Rotation from 3 to End-Effector
RRR_3E = dh_matrix(dRR(6),thetaRR(6)+data(RR,3),aRR(6),alphaRR(6));
RRR_AE = RRR_AB*RRR_B0*RRR_01*RRR_12*RRR_23*RRR_3E;

%% Rear Left Leg
alphaRL = [pi/2 0 0 -pi/2 0 0];
thetaRL = [pi/2 0 0 -pi/2 0 0];
dRL = [0 -bl/2 0 0 0 0];
aRL = [bw/2 0 l0 0 l1 l2];
RL = 60;

% Rotation from Origin (A) to Base (B) 
RRL_AB = dh_matrix(dRL(1),thetaRL(1),aRL(1),alphaRL(1));

% Rotation from B to 0 : Hip A/A
RRL_B0 = dh_matrix(dRL(2),thetaRL(2),aRL(2),alphaRL(2));
RRL_A0 = RRL_AB*RRL_B0;

% Rotation from 0 to 1
RRL_01 = dh_matrix(dRL(3),thetaRL(3)+data(RL,1),aRL(3),alphaRL(3));
RRL_A1 = RRL_AB*RRL_B0*RRL_01;

% Rotation from 1 to 2 : Hip F/E
RRL_12 = dh_matrix(dRL(4),thetaRL(4),aRL(4),alphaRL(4));
RRL_A2 = RRL_AB*RRL_B0*RRL_01*RRL_12;

% Rotation from 2 to 3 : Knee
RRL_23 = dh_matrix(dRL(5),thetaRL(5)+data(RL,2),aRL(5),alphaRL(5));
RRL_A3 = RRL_AB*RRL_B0*RRL_01*RRL_12*RRL_23;

% Rotation from 3 to End-Effector
RRL_3E = dh_matrix(dRL(6),thetaRL(6)+data(RL,3),aRL(6),alphaRL(6));
RRL_AE = RRL_AB*RRL_B0*RRL_01*RRL_12*RRL_23*RRL_3E;

%% Plot base frames
quiver3(0, 0, zB, 1, 0, 0, 0.1, 'r','linewidth', 1.5)
hold on
quiver3(0, 0, zB, 0, 1, 0, 0.1, 'g','linewidth', 1.5)
hold on
quiver3(0, 0, zB, 0, 0, 1, 0.1, 'b','linewidth', 1.5)
hold on
plot3(pB(1), pB(2), pB(3),'kx','LineWidth',2) 
hold on

% % Frame 0 for Legs on the Left Side 
% quiver3(RRL_B0(1,4),RRL_B0(2,4),RRL_B0(3,4),RRL_B0(1,1),RRL_B0(2,1),RRL_B0(3,1),0.05,'r','linewidth',1.5);
% hold on
% quiver3(RRL_B0(1,4),RRL_B0(2,4),RRL_B0(3,4),RRL_B0(1,2),RRL_B0(2,2),RRL_B0(3,2),0.05,'g','linewidth',1.5);
% hold on
% quiver3(RRL_B0(1,4),RRL_B0(2,4),RRL_B0(3,4),RRL_B0(1,3),RRL_B0(2,3),RRL_B0(3,3),0.05,'b','linewidth',1.5);
% 
% % Frame 0 for Legs on the Right Side 
% quiver3(RRR_B0(1,4),RRR_B0(2,4),RRR_B0(3,4),RRR_B0(1,1),RRR_B0(2,1),RRR_B0(3,1),0.05,'r','linewidth',1.5);
% hold on
% quiver3(RRR_B0(1,4),RRR_B0(2,4),RRR_B0(3,4),RRR_B0(1,2),RRR_B0(2,2),RRR_B0(3,2),0.05,'g','linewidth',1.5);
% hold on
% quiver3(RRR_B0(1,4),RRR_B0(2,4),RRR_B0(3,4),RRR_B0(1,3),RRR_B0(2,3),RRR_B0(3,3),0.05,'b','linewidth',1.5);

%% Hip Abbduction/Adduction of Each Leg
% Front Right
quiver3(RFR_A0(1,4),RFR_A0(2,4),RFR_A0(3,4),RFR_A0(1,1),RFR_A0(2,1),RFR_A0(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RFR_A0(1,4),RFR_A0(2,4),RFR_A0(3,4),RFR_A0(1,2),RFR_A0(2,2),RFR_A0(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RFR_A0(1,4),RFR_A0(2,4),RFR_A0(3,4),RFR_A0(1,3),RFR_A0(2,3),RFR_A0(3,3),0.05,'b','linewidth',1.5);
hold on
plot3(RFR_A0(1,4),RFR_A0(2,4),RFR_A0(3,4),'o', 'linewidth', 1.5,'MarkerFaceColor','#D95319','MarkerEdgeColor','#D95319');

% Front Left
quiver3(RFL_A0(1,4),RFL_A0(2,4),RFL_A0(3,4),RFL_A0(1,1),RFL_A0(2,1),RFL_A0(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RFL_A0(1,4),RFL_A0(2,4),RFL_A0(3,4),RFL_A0(1,2),RFL_A0(2,2),RFL_A0(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RFL_A0(1,4),RFL_A0(2,4),RFL_A0(3,4),RFL_A0(1,3),RFL_A0(2,3),RFL_A0(3,3),0.05,'b','linewidth',1.5);
hold on
plot3(RFL_A0(1,4),RFL_A0(2,4),RFL_A0(3,4),'o', 'linewidth', 1.5,'MarkerFaceColor','#D95319','MarkerEdgeColor','#D95319');

% Rear Right
quiver3(RRR_A0(1,4),RRR_A0(2,4),RRR_A0(3,4),RRR_A0(1,1),RRR_A0(2,1),RRR_A0(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RRR_A0(1,4),RRR_A0(2,4),RRR_A0(3,4),RRR_A0(1,2),RRR_A0(2,2),RRR_A0(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RRR_A0(1,4),RRR_A0(2,4),RRR_A0(3,4),RRR_A0(1,3),RRR_A0(2,3),RRR_A0(3,3),0.05,'b','linewidth',1.5);
hold on
plot3(RRR_A0(1,4),RRR_A0(2,4),RRR_A0(3,4),'o', 'linewidth', 1.5,'MarkerFaceColor','#D95319','MarkerEdgeColor','#D95319');

% Rear Left
quiver3(RRL_A0(1,4),RRL_A0(2,4),RRL_A0(3,4),RRL_A0(1,1),RRL_A0(2,1),RRL_A0(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RRL_A0(1,4),RRL_A0(2,4),RRL_A0(3,4),RRL_A0(1,2),RRL_A0(2,2),RRL_A0(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RRL_A0(1,4),RRL_A0(2,4),RRL_A0(3,4),RRL_A0(1,3),RRL_A0(2,3),RRL_A0(3,3),0.05,'b','linewidth',1.5);
hold on
plot3(RRL_A0(1,4),RRL_A0(2,4),RRL_A0(3,4),'o', 'linewidth', 1.5,'MarkerFaceColor','#D95319','MarkerEdgeColor','#D95319');

%% Hip Flexion/Extension of Each Leg
% Front Right
quiver3(RFR_A2(1,4),RFR_A2(2,4),RFR_A2(3,4),RFR_A2(1,1),RFR_A2(2,1),RFR_A2(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RFR_A2(1,4),RFR_A2(2,4),RFR_A2(3,4),RFR_A2(1,2),RFR_A2(2,2),RFR_A2(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RFR_A2(1,4),RFR_A2(2,4),RFR_A2(3,4),RFR_A2(1,3),RFR_A2(2,3),RFR_A2(3,3),0.05,'b','linewidth',1.5);
hold on 
plot3(RFR_A2(1,4),RFR_A2(2,4),RFR_A2(3,4),'o', 'linewidth', 1.5,'MarkerFaceColor','#EDB120','MarkerEdgeColor','#EDB120');

% Front Left
quiver3(RFL_A2(1,4),RFL_A2(2,4),RFL_A2(3,4),RFL_A2(1,1),RFL_A2(2,1),RFL_A2(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RFL_A2(1,4),RFL_A2(2,4),RFL_A2(3,4),RFL_A2(1,2),RFL_A2(2,2),RFL_A2(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RFL_A2(1,4),RFL_A2(2,4),RFL_A2(3,4),RFL_A2(1,3),RFL_A2(2,3),RFL_A2(3,3),0.05,'b','linewidth',1.5);
hold on
plot3(RFL_A2(1,4),RFL_A2(2,4),RFL_A2(3,4),'o', 'linewidth', 1.5,'MarkerFaceColor','#EDB120','MarkerEdgeColor','#EDB120');

% Rear Right
quiver3(RRR_A2(1,4),RRR_A2(2,4),RRR_A2(3,4),RRR_A2(1,1),RRR_A2(2,1),RRR_A2(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RRR_A2(1,4),RRR_A2(2,4),RRR_A2(3,4),RRR_A2(1,2),RRR_A2(2,2),RRR_A2(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RRR_A2(1,4),RRR_A2(2,4),RRR_A2(3,4),RRR_A2(1,3),RRR_A2(2,3),RRR_A2(3,3),0.05,'b','linewidth',1.5);
hold on
plot3(RRR_A2(1,4),RRR_A2(2,4),RRR_A2(3,4),'o', 'linewidth', 1.5,'MarkerFaceColor','#EDB120','MarkerEdgeColor','#EDB120');

% Rear Left
quiver3(RRL_A2(1,4),RRL_A2(2,4),RRL_A2(3,4),RRL_A2(1,1),RRL_A2(2,1),RRL_A2(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RRL_A2(1,4),RRL_A2(2,4),RRL_A2(3,4),RRL_A2(1,2),RRL_A2(2,2),RRL_A2(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RRL_A2(1,4),RRL_A2(2,4),RRL_A2(3,4),RRL_A2(1,3),RRL_A2(2,3),RRL_A2(3,3),0.05,'b','linewidth',1.5);
hold on
plot3(RRL_A2(1,4),RRL_A2(2,4),RRL_A2(3,4),'o', 'linewidth', 1.5,'MarkerFaceColor','#EDB120','MarkerEdgeColor','#EDB120');

%% Knee Flexion/Extension of Each Leg
% Front Right
quiver3(RFR_A3(1,4),RFR_A3(2,4),RFR_A3(3,4),RFR_A3(1,1),RFR_A3(2,1),RFR_A3(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RFR_A3(1,4),RFR_A3(2,4),RFR_A3(3,4),RFR_A3(1,2),RFR_A3(2,2),RFR_A3(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RFR_A3(1,4),RFR_A3(2,4),RFR_A3(3,4),RFR_A3(1,3),RFR_A3(2,3),RFR_A3(3,3),0.05,'b','linewidth',1.5);
hold on 
plot3(RFR_A3(1,4),RFR_A3(2,4),RFR_A3(3,4),'mo', 'linewidth', 2,'MarkerFaceColor','m');

% Front Left
quiver3(RFL_A3(1,4),RFL_A3(2,4),RFL_A3(3,4),RFL_A3(1,1),RFL_A3(2,1),RFL_A3(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RFL_A3(1,4),RFL_A3(2,4),RFL_A3(3,4),RFL_A3(1,2),RFL_A3(2,2),RFL_A3(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RFL_A3(1,4),RFL_A3(2,4),RFL_A3(3,4),RFL_A3(1,3),RFL_A3(2,3),RFL_A3(3,3),0.05,'b','linewidth',1.5);
hold on 
plot3(RFL_A3(1,4),RFL_A3(2,4),RFL_A3(3,4),'mo', 'linewidth', 2,'MarkerFaceColor','m');

% Rear Right
quiver3(RRR_A3(1,4),RRR_A3(2,4),RRR_A3(3,4),RRR_A3(1,1),RRR_A3(2,1),RRR_A3(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RRR_A3(1,4),RRR_A3(2,4),RRR_A3(3,4),RRR_A3(1,2),RRR_A3(2,2),RRR_A3(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RRR_A3(1,4),RRR_A3(2,4),RRR_A3(3,4),RRR_A3(1,3),RRR_A3(2,3),RRR_A3(3,3),0.05,'b','linewidth',1.5);
hold on 
plot3(RRR_A3(1,4),RRR_A3(2,4),RRR_A3(3,4),'mo', 'linewidth', 2,'MarkerFaceColor','m');

% Rear Left
quiver3(RRL_A3(1,4),RRL_A3(2,4),RRL_A3(3,4),RRL_A3(1,1),RRL_A3(2,1),RRL_A3(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RRL_A3(1,4),RRL_A3(2,4),RRL_A3(3,4),RRL_A3(1,2),RRL_A3(2,2),RRL_A3(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RRL_A3(1,4),RRL_A3(2,4),RRL_A3(3,4),RRL_A3(1,3),RRL_A3(2,3),RRL_A3(3,3),0.05,'b','linewidth',1.5);
hold on 
plot3(RRL_A3(1,4),RRL_A3(2,4),RRL_A3(3,4),'mo', 'linewidth', 2,'MarkerFaceColor','m');

%% End-Effect (foot) of Each Leg
% Front Right
quiver3(RFR_AE(1,4),RFR_AE(2,4),RFR_AE(3,4),RFR_AE(1,1),RFR_AE(2,1),RFR_AE(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RFR_AE(1,4),RFR_AE(2,4),RFR_AE(3,4),RFR_AE(1,2),RFR_AE(2,2),RFR_AE(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RFR_AE(1,4),RFR_AE(2,4),RFR_AE(3,4),RFR_AE(1,3),RFR_AE(2,3),RFR_AE(3,3),0.05,'b','linewidth',1.5);
hold on 
plot3(RFR_AE(1,4),RFR_AE(2,4),RFR_AE(3,4),'ko', 'linewidth', 2,'MarkerFaceColor','k');
hold on
plot3([RFR_A2(1,4) RFR_A3(1,4) RFR_AE(1,4)], [RFR_A2(2,4) RFR_A3(2,4) RFR_AE(2,4)], [RFR_A2(3,4) RFR_A3(3,4) RFR_AE(3,4)],'Color',gray, 'LineWidth',3)

% Front Left
quiver3(RFL_AE(1,4),RFL_AE(2,4),RFL_AE(3,4),RFL_AE(1,1),RFL_AE(2,1),RFL_AE(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RFL_AE(1,4),RFL_AE(2,4),RFL_AE(3,4),RFL_AE(1,2),RFL_AE(2,2),RFL_AE(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RFL_AE(1,4),RFL_AE(2,4),RFL_AE(3,4),RFL_AE(1,3),RFL_AE(2,3),RFL_AE(3,3),0.05,'b','linewidth',1.5);
hold on
plot3(RFL_AE(1,4),RFL_AE(2,4),RFL_AE(3,4),'ko', 'linewidth', 2,'MarkerFaceColor','k');
hold on
plot3([RFL_A2(1,4) RFL_A3(1,4) RFL_AE(1,4)], [RFL_A2(2,4) RFL_A3(2,4) RFL_AE(2,4)], [RFL_A2(3,4) RFL_A3(3,4) RFL_AE(3,4)],'Color', gray, 'LineWidth',3)

% Rear Right
quiver3(RRR_AE(1,4),RRR_AE(2,4),RRR_AE(3,4),RRR_AE(1,1),RRR_AE(2,1),RRR_AE(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RRR_AE(1,4),RRR_AE(2,4),RRR_AE(3,4),RRR_AE(1,2),RRR_AE(2,2),RRR_AE(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RRR_AE(1,4),RRR_AE(2,4),RRR_AE(3,4),RRR_AE(1,3),RRR_AE(2,3),RRR_AE(3,3),0.05,'b','linewidth',1.5);
hold on
plot3(RRR_AE(1,4),RRR_AE(2,4),RRR_AE(3,4),'ko', 'linewidth', 2,'MarkerFaceColor','k');
hold on
plot3([RRR_A2(1,4) RRR_A3(1,4) RRR_AE(1,4)], [RRR_A2(2,4) RRR_A3(2,4) RRR_AE(2,4)], [RRR_A2(3,4) RRR_A3(3,4) RRR_AE(3,4)],'Color', gray, 'LineWidth',3)

% Rear Left
quiver3(RRL_AE(1,4),RRL_AE(2,4),RRL_AE(3,4),RRL_AE(1,1),RRL_AE(2,1),RRL_AE(3,1),0.05,'r','linewidth',1.5);
hold on
quiver3(RRL_AE(1,4),RRL_AE(2,4),RRL_AE(3,4),RRL_AE(1,2),RRL_AE(2,2),RRL_AE(3,2),0.05,'g','linewidth',1.5);
hold on
quiver3(RRL_AE(1,4),RRL_AE(2,4),RRL_AE(3,4),RRL_AE(1,3),RRL_AE(2,3),RRL_AE(3,3),0.05,'b','linewidth',1.5);
hold on
plot3(RRL_AE(1,4),RRL_AE(2,4),RRL_AE(3,4),'ko', 'linewidth', 2,'MarkerFaceColor','k');
hold on
plot3([RRL_A2(1,4) RRL_A3(1,4) RRL_AE(1,4)], [RRL_A2(2,4) RRL_A3(2,4) RRL_AE(2,4)], [RRL_A2(3,4) RRL_A3(3,4) RRL_AE(3,4)],'Color', gray,'LineWidth',3)

% gotta be a more effective way to print the legend
legend('','','','Origin','','','','','','','','Hip : Abbduction/Adduction','','','','','', ...
    '','','','','','','','','','','Hip : Flexion/Extension','','','','','','','','','', ...
    '','','','','','','','','','','Knee','','','','','','','','','','','','', ...
    '','','','','','','','','','','Foot','Leg')
xlabel('x'); ylabel('y'); zlabel('z');
ylim([-0.3 0.3])
grid on

