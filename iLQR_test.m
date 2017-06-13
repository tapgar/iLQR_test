
clear
close all

x_targ = [pi/2; pi/2; 0; 0];

traj_opt = iLQR(50, x_targ);

x0 = zeros(4,1);

traj_opt.RuniLQR(x0);