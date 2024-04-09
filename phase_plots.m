%% Phase Plots

clear all
close all

joint_q = load('../go1/Data/walking_joint_q.txt');

fc = 10;
fs = 500; % Hz 
[b,a] = butter(2,2*fc/fs);  % digital filter
knee_q_filt = filtfilt(b,a,joint_q);

knee_dq = gradient(knee_q_filt(:,3),1/500);

figure(1)
plot(knee_q_filt(1000:end,3), knee_dq(1000:end));
xlabel('Angle (rad)')
ylabel('Velocity (rad/s)')

figure(2)
hold on
for i = 1000:length(knee_q_filt)
    plot(knee_q_filt(i,3),knee_dq(i),'o','MarkerEdgeColor','#77AC30')
    xlim([-1.85 -1.5])
    ylim([-3 3])
    xlabel('Angle (rad)')
    ylabel('Velocity (rad/s)')
    drawnow
    pause(0.01)
end
hold off


