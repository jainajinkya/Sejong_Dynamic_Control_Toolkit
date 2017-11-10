clear all
clc 
close all

%% 
fn_path = '~/Sejong_Dynamic_Control_Toolkit/Addition/Matlab_codes/functions';
addpath(fn_path)

data_path = '~/Sejong_Dynamic_Control_Toolkit/experiment_data_check';
read_data_simple_file;
fig = fn_open_figures(4);

min_length = length(t) - 0;
x = t(1:min_length);
%% Draw Figure
% Torque
figure(fig(1))

for i = 1:3
  tmp(i) = subplot(3,1,i);
  plot(x, torque(i,1:min_length) ,'b-', 'linewidth',1.0);
end
title(tmp(1), 'Torque (right)');
xlabel('Time (sec)','fontsize', 12);

figure(fig(2))
for i = 1:3
      tmp(i) = subplot(3,1,i);
  plot(x, torque(i+3,1:min_length) ,'b-', 'linewidth',1.2);
end
xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'Torque (left)');

%% Reaction Force

figure(fig(3))

for i = 1:3
  tmp(i) = subplot(3,1,i);
  plot(x, rforce(i,1:min_length) ,'b-', 'linewidth',1.0);
end
title(tmp(1), 'rforce (right)');
xlabel('Time (sec)','fontsize', 12);

figure(fig(4))
for i = 1:3
      tmp(i) = subplot(3,1,i);
  plot(x, rforce(i+3,1:min_length) ,'b-', 'linewidth',1.2);
end
xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'rforce (left)');


