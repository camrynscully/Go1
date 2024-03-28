%% Go1 - Single Leg Simulation

clear all
close all

gray = [.7 .7 .7];

% data = load('walking_soft_joint_positions.txt');
data = load('walking_joint_q.txt');
 
% length of each link [m]
bw = 0.0935;
bl = 0.3762;
l0 = 0.08;
l1 = 0.213;
l2 = 0.213;

% FR Leg
alphaFR = [pi/2 0 0 -pi/2 0 0];
thetaFR = [pi/2 0 0 -pi/2 0 0];
dFR = [0 bl/2 0 0 0 0];
aFR = [-bw/2 0 -l0 0 l1 l2];
FR = 57; endIndex = FR*8;

% FL Leg
% alphaFL = [pi/2 0 0 -pi/2 0 0];
% thetaFL = [pi/2 0 0 -pi/2 0 0];
% dFL = [0 bl/2 0 0 0 0];
% aFRL = [bw/2 0 l0 0 l1 l2];
% FL = 58; endIndex = FL*8;

% RR Leg
% alphaRR = [pi/2 0 0 -pi/2 0 0];
% thetaRR = [pi/2 0 0 -pi/2 0 0];
% dRR = [0 -bl/2 0 0 0 0];
% aRR = [-bw/2 0 -l0 0 l1 l2];
% RR = 59; endIndex = RR*8;

% RL Leg
% alphaRL = [pi/2 0 0 -pi/2 0 0];
% thetaRL = [pi/2 0 0 -pi/2 0 0];
% dRL = [0 -bl/2 0 0 0 0];
% aRL = [bw/2 0 l0 0 l1 l2];
% RL = 60; endIndex = RL*8;

figure
for i = FR:4:endIndex
    
    % Rotation & Translation from Base to B2 
    RFR_B0 = dh_matrix(dFR(1),thetaFR(1),aFR(1),alphaFR(1));
    
    % Translation from B2 to 0 : Hip A/A
    RFR_01A = dh_matrix(dFR(2),thetaFR(2),aFR(2),alphaFR(2));
    RFR_B1A = RFR_B0*RFR_01A;
    
    % Rotation & Translation from 0 to 1
    RFR_1A1B = dh_matrix(dFR(3),thetaFR(3)+data(i,1),aFR(3),alphaFR(3));
    RFR_B1B = RFR_B0*RFR_01A*RFR_1A1B;
    
    % Rotation & Translation from 1 to 2 : Hip F/E
    RFR_1B2 = dh_matrix(dFR(4),thetaFR(4),aFR(4),alphaFR(4));
    RFR_B2 = RFR_B0*RFR_01A*RFR_1A1B*RFR_1B2;
    
    % Rotation & Translation from 2 to 3 : Knee 
    RFR_23 = dh_matrix(dFR(5),thetaFR(5)+data(i,2),aFR(5),alphaFR(5));
    RFR_B3 = RFR_B0*RFR_01A*RFR_1A1B*RFR_1B2*RFR_23;
    
    % Rotation & Translation from 3 to End-Effector
    RFR_3E = dh_matrix(dFR(6),thetaFR(6)+data(i,3),aFR(6),alphaFR(6));
    RFR_BE = RFR_B0*RFR_01A*RFR_1A1B*RFR_1B2*RFR_23*RFR_3E;

    plot3([RFR_B1A(1,4) RFR_B2(1,4) RFR_B3(1,4) RFR_BE(1,4)], [RFR_B1A(2,4) RFR_B2(2,4) RFR_B3(2,4) RFR_BE(2,4)], ...
        [RFR_B1A(3,4) RFR_B2(3,4) RFR_B3(3,4) RFR_BE(3,4)] ,'Color', gray, 'LineWidth', 4);
    hold on
    plot3(RFR_B1A(1,4),RFR_B1A(2,4),RFR_B1A(3,4),'o','LineWidth',1.5,'MarkerFaceColor','#D95319','MarkerEdgeColor','#D95319')
    hold on
    plot3(RFR_B2(1,4),RFR_B2(2,4),RFR_B2(3,4),'o','LineWidth',1.5,'MarkerFaceColor','#EDB120','MarkerEdgeColor','#EDB120')
    hold on
    plot3(RFR_B3(1,4),RFR_B3(2,4),RFR_B3(3,4),'mo','LineWidth',1.5,'MarkerFaceColor','m')
    hold on
    plot3(RFR_BE(1,4),RFR_BE(2,4),RFR_BE(3,4),'ko','LineWidth',1.5,'MarkerFaceColor','k')
    xlim([-0.4 0.5]); ylim([-0.4 0.5]); zlim([-0.5 0.1])
    xlabel('x'); ylabel('y'); zlabel('z')
    legend('Leg','Hip : Abduction/Adduction','Hip : Flexion/Extension','Knee','Foot')
    title('FR Leg')
    grid on

    hold on
    pause(0.0001);

    if i < endIndex - 4
        cla
    end
end






