%% Foot Force Plots

clear all
close all

% force_running = load('running_foot_forces.txt');
force_walking = load('../Data/foot_forces_walk_soft_rigid_04_03_24.txt');

force_standing_soft = load('../Data/foot_forces_standing_soft_04_03_24.txt');
force_standing_rigid = load('../Data/foot_forces_standing_rigid_04_03_24.txt');

fc = 10;
fs = 500; % Hz 
[b,a] = butter(2,2*fc/fs);  % digital filter

force_walking_filt = filtfilt(b,a,force_walking);
force_standing_soft_filt = filtfilt(b,a,force_standing_soft);
force_standing_rigid_filt = filtfilt(b,a,force_standing_rigid);

FR_min_rigid = min(force_walking_filt(1:3000,1));
FR_max_rigid = max(force_walking_filt(1:3000,1));
FR_min_soft = min(force_walking_filt(3001:end,1));
FR_max_soft = max(force_walking_filt(3001:end,1));

FL_min_rigid = min(force_walking_filt(1:3000,2));
FL_max_rigid = max(force_walking_filt(1:3000,2));
FL_min_soft = min(force_walking_filt(3001:end,2));
FL_max_soft = max(force_walking_filt(3001:end,2));

RL_min_rigid = min(force_walking_filt(1:3000,3));
RL_max_rigid = max(force_walking_filt(1:3000,3));
RL_min_soft = min(force_walking_filt(3001:end,3));
RL_max_soft = max(force_walking_filt(3001:end,3));

RR_min_rigid = min(force_walking_filt(1:3000,4));
RR_max_rigid = max(force_walking_filt(1:3000,4));
RR_min_soft = min(force_walking_filt(3001:end,4));
RR_max_soft = max(force_walking_filt(3001:end,4));

nw = length(force_walking_filt);
tw = (1/fs)*(0:nw-1);

ns = length(force_standing_soft_filt);
ts = (1/fs)*(0:ns-1);

nr = length(force_standing_rigid_filt);
tr = (1/fs)*(0:nr-1);

fig1 = figure(1);
plot(tw, force_walking)
xlabel('\textbf{Time (s)}','Interpreter','Latex')
ylabel('\textbf{Force [N]}','Interpreter','Latex')
legend('\textbf{Front Right}','\textbf{Front Left}', ...
    '\textbf{Rear Right}', '\textbf{Rear Left}','Interpreter','Latex')
title('Foot Forces: Unfiltered','FontWeight','bold')
grid on

% make axis ticks bold
temp = get(gca,'XTick');
xticklabels = cellstr(num2str(temp(:))); % Convert numeric tick values to cell array of strings
set(gca, 'XTickLabel', xticklabels, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');

temp = get(gca,'YTick');
yticklabels = cellstr(num2str(temp(:))); % Convert numeric tick values to cell array of strings
set(gca, 'YTickLabel', yticklabels, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
% set(gca, 'FontSize', 12); % Adjust the font size if necessary

fig2 = figure(2);
plot(tw, force_walking_filt)
xlabel('\textbf{Time (s)}','Interpreter','Latex')
ylabel('\textbf{Force [N]}','Interpreter','Latex')
legend('\textbf{Front Right}','\textbf{Front Left}', ...
    '\textbf{Rear Right}', '\textbf{Rear Left}','Interpreter','Latex')
title('Foot Forces: Rigid to Soft','FontWeight','bold')
grid on

% make axis ticks bold
temp = get(gca,'XTick');
xticklabels = cellstr(num2str(temp(:))); % Convert numeric tick values to cell array of strings
set(gca, 'XTickLabel', xticklabels, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');

temp = get(gca,'YTick');
yticklabels = cellstr(num2str(temp(:))); % Convert numeric tick values to cell array of strings
set(gca, 'YTickLabel', yticklabels, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
% set(gca, 'FontSize', 12); % Adjust the font size if necessary

fig3 = figure(3);
plot(ts, force_standing_soft)
xlabel('\textbf{Time (s)}','Interpreter','Latex')
ylabel('\textbf{Force [N]}','Interpreter','Latex')
legend('\textbf{Front Right}','\textbf{Front Left}', ...
    '\textbf{Rear Right}', '\textbf{Rear Left}','Interpreter','Latex')
title('Foot Forces: Soft','FontWeight','bold')

% make axis ticks bold
temp = get(gca,'XTick');
xticklabels = cellstr(num2str(temp(:))); % Convert numeric tick values to cell array of strings
set(gca, 'XTickLabel', xticklabels, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');

temp = get(gca,'YTick');
yticklabels = cellstr(num2str(temp(:))); % Convert numeric tick values to cell array of strings
set(gca, 'YTickLabel', yticklabels, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
% set(gca, 'FontSize', 12); % Adjust the font size if necessary

fig4 = figure(4);
plot(tr, force_standing_rigid)
xlabel('\textbf{Time (s)}','Interpreter','Latex')
ylabel('\textbf{Force [N]}','Interpreter','Latex')
legend('\textbf{Front Right}','\textbf{Front Left}', ...
    '\textbf{Rear Right}', '\textbf{Rear Left}','Interpreter','Latex')
title('Foot Forces: Rigid','FontWeight','bold')

% make axis ticks bold
temp = get(gca,'XTick');
xticklabels = cellstr(num2str(temp(:))); % Convert numeric tick values to cell array of strings
set(gca, 'XTickLabel', xticklabels, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');

temp = get(gca,'YTick');
yticklabels = cellstr(num2str(temp(:))); % Convert numeric tick values to cell array of strings
set(gca, 'YTickLabel', yticklabels, 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
% set(gca, 'FontSize', 12); % Adjust the font size if necessary

% figure(3)
% plot(points2(1:1500), force_walking(1:1500,:))
% % xlabel('k')
% % ylabel('Volts (mV?)')
% legend('Front Right','Front Left', 'Rear Right', 'Rear Left')
% title('Foot Force Measurements over Rigid')

output = "foot_forces";
% print(fig1,output+"_walking_rigid_soft_raw",'-dpng','-r300');
% savefig(fig1,output + "_walking_rigid_soft_raw"+".fig");
% 
% print(fig2,output+"_walking_rigid_soft_filt",'-dpng','-r300');
% savefig(fig2,output + "_walking_rigid_soft_filt"+".fig");

print(fig3,output + "_standing_soft",'-dpng','-r300');
savefig(fig3,output + "_standing_soft" + ".fig");

print(fig4,output + "_standing_rigid",'-dpng','-r300');
savefig(fig4,output + "_standing_rigid" + ".fig");


