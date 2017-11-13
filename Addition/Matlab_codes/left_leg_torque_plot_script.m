for i = 1:3
  tmp(i) = subplot(3,1,i);
  plot(x, torque(i+3,1:min_length) ,'b-', 'linewidth',1.2);
  phase_drawing_script;
end

xlabel('Time (sec)','fontsize', 12);
title(tmp(1), 'Torque (left)');
