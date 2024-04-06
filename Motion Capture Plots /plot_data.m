%% Plot data from OptiTrack

clear all
close all

pos = load("../Data/position_X_Z_Y_04_04_24_new_policy.txt");  % z = forward, x = lateral, y = vertical
% pos = load("../Data/position_X_Z_Y_04_04_24_baseline.txt");

% flip the y and z positions so it is consisent for velocity, accel calcuations
position = [pos(:,1) pos(:,3) pos(:,2)];

% % determine 
% t0 = 25;
% tf = 110;
% switch1 = 240*t0;
% switch2 = 240*tf;
% position = position(switch1:switch2,:);

fc = 2;  % look into cutoff frequency
fs = 240; % Hz
[b,a] = butter(2,2*fc/fs);  % digital filter
position_filt = filtfilt(b,a,position);

n = length(position_filt);
t = (1/fs)*(0:n-1); 

velocity(:,1) = gradient(position_filt(:,1),1/fs);
velocity(:,2) = gradient(position_filt(:,2),1/fs); 
velocity(:,3) = gradient(position_filt(:,3),1/fs);

% acceleration
acceleration(:,1) = gradient(velocity(:,1),1/fs);
acceleration(:,2) = gradient(velocity(:,2),1/fs);
acceleration(:,3) = gradient(velocity(:,3),1/fs);

% jerk
jerk(:,1) = gradient(acceleration(:,1),1/fs);
jerk(:,2) = gradient(acceleration(:,2),1/fs);
jerk(:,3) = gradient(acceleration(:,3),1/fs);

% 3D position plot
fig1 = figure(); figure(fig1)
plot3(position(:,1), position(:,3), position(:,2));
hold on
plot3(position(1,1), position(1,3), position(1,2),'go');
hold on
plot3(position(end,1), position(end,3), position(end,2),'ro');
legend('x','Start', 'End');
xlabel('x'); ylabel('y'); zlabel('z');
title('Position (m)')
grid on

% plot(t, position_filt(:,1));
% hold on
% plot(t, position_filt(:,2));
% hold on
% plot(t, position_filt(:,3));
% xlabel('Time (s)'); ylabel('Position (m)'); 
% legend('Px','Py','Pz')
% title('Position (m)')
% grid on

for i = 1:3  % make axis ticks bold
    subplot(3,1,i);

    temp = get(gca,'XTick');
    xticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'XTickLabel', xticklabels, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    
    temp = get(gca,'YTick');
    yticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'YTickLabel', yticklabels, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    % set(gca, 'FontSize', 12); % Adjust the font size if necessary
end

fig2 = figure(2);
subplot(3, 1, 1);
plot(t, position_filt(:,1));
ylabel('$\bf{P_{x}}$ \bf{(m)}', 'Interpreter', 'latex');
title('$\textbf{Position vs. Time}$', 'Interpreter', 'latex');

subplot(3, 1, 2);
plot(t, position_filt(:,2));
ylabel('$\bf{P_{y}}$ \bf{(m)}', 'Interpreter', 'latex');

subplot(3, 1, 3);
plot(t, position_filt(:,3))
% hold on 
% plot(t0,position_filt(switch1,3),'ro')
% hold on
% plot(tf,position_filt(switch2,3),'ro')
xlabel('$\textbf{Time (s)}$', 'Interpreter', 'latex');
ylabel('$\bf{P_{z}}$ \bf{(m)}', 'Interpreter', 'latex');

for i = 1:3  % make axis ticks bold
    subplot(3,1,i);

    temp = get(gca,'XTick');
    xticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'XTickLabel', strcat('\bf{', xticklabels, '}'),'FontSize', 12);
    
    temp = get(gca,'YTick');
    yticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'YTickLabel', strcat('\bf{', yticklabels, '}'), 'FontSize', 12);
    set(gca, 'TickLabelInterpreter', 'latex'); % Set tick label interpreter to LaTeX
    grid on
end

% Plot Velocity
fig3 = figure(3);
subplot(3, 1, 1);
plot(t, velocity(:,1))
ylabel('$\bf{V_{x}}$ \bf{(m/s)}', 'Interpreter', 'latex');
title('$\textbf{Velocity vs. Time}$', 'Interpreter', 'latex');

subplot(3, 1, 2);
plot(t, velocity(:,2))
ylabel('$\bf{V_{y}}$ \bf{(m/s)}', 'Interpreter', 'latex');

subplot(3, 1, 3);
plot(t, velocity(:,3))
% hold on
% plot(t0,velocity(switch1,3),'ro')
% hold on
% plot(tf,velocity(switch2,3),'ro')
xlabel('$\textbf{Time (s)}$', 'Interpreter', 'latex');
% xlabel('$\bf{Time (s)}', 'Interpreter', 'latex');
ylabel('$\bf{V_{z}}$ \bf{(m/s)}', 'Interpreter', 'latex');

for i = 1:3  % make axis ticks bold
    subplot(3,1,i);

    temp = get(gca,'XTick');
    xticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'XTickLabel', strcat('\bf{', xticklabels, '}'), 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    
    temp = get(gca,'YTick');
    yticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'YTickLabel',  strcat('\bf{', yticklabels, '}'), 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    set(gca, 'TickLabelInterpreter', 'latex'); % Set tick label interpreter to LaTeX
    grid on
end

% Plot Acceleration
fig4 = figure(4);
subplot(3, 1, 1);
plot(t, acceleration(:,1));
ylabel('$\bf{A_{x}}$ \bf{(m/s$^2$)}', 'Interpreter', 'latex');
title('$\textbf{Acceleration vs. Time}$', 'Interpreter', 'latex');

subplot(3, 1, 2);
plot(t, acceleration(:,2));
ylabel('$\bf{A_{y}}$ \bf{(m/s$^2$)}', 'Interpreter', 'latex');

subplot(3, 1, 3);
plot(t, acceleration(:,3));
xlabel('Time (s)');
% xlabel('$\bf{Time (s)}', 'Interpreter', 'latex');
ylabel('$\bf{A_{z}}$ \bf{(m/s$^2$)}', 'Interpreter', 'latex');

for i = 1:3  % make axis ticks bold
    subplot(3,1,i);

    temp = get(gca,'XTick');
    xticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'XTickLabel', strcat('\bf{', xticklabels, '}'), 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');

    temp = get(gca,'YTick');
    yticklabels = cellstr(num2str(temp(:)));
    set(gca, 'YTickLabel', strcat('\bf{', yticklabels, '}'), 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    set(gca, 'TickLabelInterpreter', 'latex'); % Set tick label interpreter to LaTeX
    grid on
end


% Plot Jerk
fig5 = figure(5);
subplot(3, 1, 1);
plot(t, jerk(:,1));
ylabel('$\bf{J_{x}}$ \bf{(m/s$^3$)}', 'Interpreter', 'latex');
title('$\textbf{Jerk vs. Time}$', 'Interpreter', 'latex');

subplot(3, 1, 2);
plot(t, jerk(:,2));
ylabel('$\bf{J_{y}}$ \bf{(m/s$^3$)}', 'Interpreter', 'latex');

subplot(3, 1, 3);
plot(t, jerk(:,3));
xlabel('$\textbf{Time (s)}$', 'Interpreter', 'latex');
ylabel('$\bf{J_{z}}$ \bf{(m/s$^3$)}', 'Interpreter', 'latex');

for i = 1:3  % make axis ticks bold
    subplot(3,1,i);

    temp = get(gca,'XTick');
    xticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'XTickLabel', strcat('\bf{', xticklabels, '}'), 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    
    temp = get(gca,'YTick');
    yticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'YTickLabel',  strcat('\bf{', yticklabels, '}'), 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    set(gca, 'TickLabelInterpreter', 'latex'); % Set tick label interpreter to LaTeX
    grid on
end



%% Orientation Plots

orientation = load("../Data/orientation_Yaw_Pitch_Roll_04_04_24_new_policy.txt");
% orientation = load("../Data/orientation_Yaw_Pitch_Roll_04_04_24_baseline.txt");

orientation = deg2rad(orientation); % convert to radians

fc = 3;
[b,a] = butter(2,2*fc/fs);  % digital filter
orientation_filt = filtfilt(b,a,orientation);

no = length(orientation);
to = (1/fs)*(0:no-1);

fig6 = figure(6);
subplot(3, 1, 1);
plot(to, orientation_filt(:,3));
ylabel('$\textbf{Roll (rad)}$', 'Interpreter', 'latex');
title('$\textbf{Orientation}$', 'Interpreter', 'latex');

subplot(3, 1, 2);
plot(to, orientation_filt(:,2));
ylabel('$\textbf{Pitch (rad)}$', 'Interpreter', 'latex');

subplot(3, 1, 3);
plot(to, orientation_filt(:,1));
xlabel('$\textbf{Time (s)}$', 'Interpreter', 'latex');
ylabel('$\textbf{Yaw (rad)}$', 'Interpreter', 'latex');

for i = 1:3  % make axis ticks bold
    subplot(3,1,i);

    temp = get(gca,'XTick');
    xticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'XTickLabel',  strcat('\bf{', xticklabels, '}'), 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    
    temp = get(gca,'YTick');
    yticklabels = cellstr(num2str(temp(:))); 
    set(gca, 'YTickLabel',  strcat('\bf{', yticklabels, '}'), 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    
    set(gca, 'TickLabelInterpreter', 'latex'); % Set tick label interpreter to LaTeX
    grid on
end

% save figures
% output = "baseline"; 
output = "new_policy";

% print(fig1,"_position",'-dpng','-r300');
% savefig(fig1,"_position"+".fig");

print(fig2,output + "_position",'-dpng','-r300');
savefig(fig2,output + "_position"+".fig");

print(fig3,output +"_velocity",'-dpng','-r300');
savefig(fig3,output + "_velocity"+".fig");

print(fig4,output+"_acceleration",'-dpng','-r300');
savefig(fig4,output + "_acceleration"+".fig");

print(fig5,output+"_jerk",'-dpng','-r300');
savefig(fig5,output + "_jerk"+".fig");

print(fig6,output + "_orientation",'-dpng','-r300');
savefig(fig6,output + "__orientation"+".fig");


%% Metrics for Acceleration and Jerk

% acceleration
min_acc_x = min(acceleration(:,1));
min_acc_y = min(acceleration(:,2));
min_acc_z = min(acceleration(:,3));

max_acc_x = max(acceleration(:,1));
max_acc_y = max(acceleration(:,2));
max_acc_z = max(acceleration(:,3));

range_acc_x = abs(max_acc_x - abs(min_acc_x));
range_acc_y = abs(max_acc_y - abs(min_acc_y));
range_acc_z = abs(max_acc_z - abs(min_acc_z));

% finding avg & std for accel
avg_acc_x = mean(abs(acceleration(:,1)));
avg_acc_y = mean(abs(acceleration(:,2)));
avg_acc_z = mean(abs(acceleration(:,3)));

std_acc_x = std(abs(acceleration(:,1)));
std_acc_y = std(abs(acceleration(:,2)));
std_acc_z = std(abs(acceleration(:,3)));

% % avg and std for just the main walking part (not starting and stopping)
% avg_acc_walk_x = mean(abs(acceleration(switch1:switch2,1)));
% avg_acc_walk_y = mean(abs(acceleration(switch1:switch2,2)));
% avg_acc_walk_z = mean(abs(acceleration(switch1:switch2,3)));
% 
% std_acc_walk_x = std(abs(acceleration(switch1:switch2,1)));
% std_acc_walk_y = std(abs(acceleration(switch1:switch2,2)));
% std_acc_walk_z = std(abs(acceleration(switch1:switch2,3)));

%% jerk
min_jerk_x = min(jerk(:,1));
min_jerk_y = min(jerk(:,2));
min_jerk_z = min(jerk(:,3));

max_jerk_x = max(jerk(:,1));
max_jerk_y = max(jerk(:,2));
max_jerk_z = max(jerk(:,3));

range_jerk_x = abs(max_jerk_x - abs(min_jerk_x));
range_jerk_y = abs(max_jerk_y - abs(min_jerk_y));
range_jerk_z = abs(max_jerk_z - abs(min_jerk_z));

% finding avg & std for jerk
avg_jerk_x = mean(abs(jerk(:,1)));
avg_jerk_y = mean(abs(jerk(:,2)));
avg_jerk_z = mean(abs(jerk(:,3)));

std_jerk_x = std(abs(jerk(:,1)));
std_jerk_y = std(abs(jerk(:,2)));
std_jerk_z = std(abs(jerk(:,3)));

% % avg and std for just the main walking part (not starting and stopping)
% avg_jerk_walk_x = mean(abs(jerk(switch1:switch2,1)));
% avg_jerk_walk_y = mean(abs(jerk(switch1:switch2,2)));
% avg_jerk_walk_z = mean(abs(jerk(switch1:switch2,3)));
% 
% std_jerk_walk_x = std(abs(jerk(switch1:switch2,1)));
% std_jerk_walk_y = std(abs(jerk(switch1:switch2,2)));
% std_jerk_walk_z = std(abs(jerk(switch1:switch2,3)));

sz = [3 3];
varNames1 = ["Acceleration","Mean","STD"];
varTypes = ["string","double","double"];
acc_values = table('Size',sz,'VariableTypes',varTypes,'VariableNames',varNames1);

acc_values(1,:) = {'Ax',avg_acc_x, std_acc_x};
acc_values(2,:) = {'Ay',avg_acc_y, std_acc_y};
acc_values(3,:) = {'Az',avg_acc_z, std_acc_z};

varNames2 = ["Jerk","Mean","STD"];
jerk_values = table('Size',sz,'VariableTypes',varTypes,'VariableNames',varNames2);

jerk_values(1,:) = {'Jx',avg_jerk_x, std_jerk_x};
jerk_values(2,:) = {'Jy',avg_jerk_y, std_jerk_y};
jerk_values(3,:) = {'Jz',avg_jerk_z, std_jerk_z};

sz3 = [6 4];
varNames3 = ["Metric","Min Value","Max Value","Range"];
varTypes3 = ["string","double","double","double"];
range_values = table('Size',sz3,'VariableTypes',varTypes3,'VariableNames',varNames3);

range_values(1,:) = {'Ax',min_acc_x,max_acc_x,range_acc_x};
range_values(2,:) = {'Ay',min_acc_y,max_acc_y,range_acc_y};
range_values(3,:) = {'Az',min_acc_z,max_acc_z,range_acc_z};
range_values(4,:) = {'Jx',min_jerk_x,max_jerk_x,range_jerk_x};
range_values(5,:) = {'Jy',min_jerk_y,max_jerk_y,range_jerk_y};
range_values(6,:) = {'Jz',min_jerk_z,max_jerk_z,range_jerk_z};

disp(acc_values);
disp(jerk_values);
disp(range_values);



