t = fn_read_file(data_path, 'time', 1);

phase = fn_read_file(data_path, 'phase', 1);

torque  = fn_read_file(data_path, 'torque',6);
q   = fn_read_file(data_path, 'config', 13);
qdot = fn_read_file(data_path, 'qdot', 12);


reaction_force = fn_read_file(data_path, 'reaction_force', 6);
com_pos = fn_read_file(data_path, 'com_pos', 3);
com_vel = fn_read_file(data_path, 'com_vel', 3);

rfoot_pos = fn_read_file(data_path, 'rfoot_pos', 3);
rfoot_vel = fn_read_file(data_path, 'rfoot_vel', 3);
lfoot_pos = fn_read_file(data_path, 'lfoot_pos', 3);
lfoot_vel = fn_read_file(data_path, 'lfoot_vel', 3);

rfoot_pos_des = fn_read_file(data_path, 'rfoot_pos_des',3);
rfoot_vel_des = fn_read_file(data_path, 'rfoot_vel_des',3);
lfoot_pos_des = fn_read_file(data_path, 'lfoot_pos_des',3);
lfoot_vel_des = fn_read_file(data_path, 'lfoot_vel_des',3);
