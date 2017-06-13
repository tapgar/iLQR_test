close all
clear

load('min_torque_path.mat');
min_t_path = xy_path;

xf = [pi/2, pi/2, 0, 0];

%pos, vel, u
x0 = [linspace(0,pi/2,50)'; linspace(0, pi/2, 50)'; zeros(200,1)];
arm = ArmModel(true);

f = @(x)arm_dircol_constraints(x, arm, xf);

%something similar to iLQR (min torque)
%min_func = @(x)(sum(x(201:end).^2));
%diff torque
%min_func = @(x)(sum(diff(x(201:250)).^2+diff(x(251:end)).^2));

%min joint jerk
min_func = @(x)(sum(diff(diff(x(101:150))).^2+diff(diff(x(151:200))).^2));

%min end effector jerk
%min_func = @(x)arm.get_ef_jerk(x);



options = optimoptions(@fmincon, 'TolFun', 0.000001, 'MaxIter', 10000, ...
                       'MaxFunEvals', 50000, 'Display', 'iter', ...
                       'DiffMinChange', 0.00001, 'Algorithm', 'sqp');

% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];

optimal = fmincon(min_func, x0, A, b, Aeq, Beq, [], [], ...
               f, options);
           
t1 = optimal(1:50);
t2 = optimal(51:100);
td1 = optimal(101:150);
td2 = optimal(151:200);

x_traj = [t1, t2, td1, td2];

u_traj = [optimal(201:250), optimal(251:end)];

arm.playback(x_traj)

xy_path = arm.get_cartesian(x_traj);

hold on
plot(xy_path(:,1),xy_path(:,2))
plot(min_t_path(:,1), min_t_path(:,2), 'r');