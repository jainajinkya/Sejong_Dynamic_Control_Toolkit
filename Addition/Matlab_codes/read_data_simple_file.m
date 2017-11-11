t = fn_read_file(data_path, 'time', 1);

phase = fn_read_file(data_path, 'phase', 1);

torque  = fn_read_file(data_path, 'torque',6);
q   = fn_read_file(data_path, 'config', 13);
qdot = fn_read_file(data_path, 'qdot', 12);

rforce = fn_read_file(data_path, 'reaction_force', 6);