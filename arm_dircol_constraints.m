function [ c, ceq ] = arm_dircol_constraints( x, arm, xf )
    
    % No nonlinear inequality constraint needed
    c = [];

    % Get the states / inputs out of the vector
    positions = x(1:100);
    vels      = x(101:200);
    u      = x(201:end);
    
    % Constrain initial position and velocity to be zero
    ceq = [positions(1); vels(1); positions(51); vels(51)];
    for i = 1 : 50 - 1
        % The state at the beginning of the time interval
        x_i = [positions(i); positions(i+50); vels(i); vels(i+50)];
        % What the state should be at the start of the next time interval
        x_n = [positions(i+1); positions(i+51); vels(i+1); vels(i+51)];
        % The time derivative of the state at the beginning of the time
        % interval
        xvec = [positions(i); positions(i+50); vels(i); vels(i+50)];
        uvec = [u(i); u(i+50)];
        xdd = arm.forward(xvec, uvec);
        
        xdot_i = [vels(i); vels(i+50); xdd];
        
        xvec = [positions(i+1); positions(i+51); vels(i+1); vels(i+51)];
        uvec = [u(i+1); u(i+51)];
        xdd = arm.forward(xvec, uvec);

        % The time derivative of the state at the end of the time interval
        xdot_n = [vels(i+1); vels(i+51); xdd];
        
        % The end state of the time interval calculated using quadrature
        xend = x_i + 0.01 * (xdot_i + xdot_n) / 2;
        % Constrain the end state of the current time interval to be
        % equal to the starting state of the next time interval
        ceq = [ceq ; x_n - xend];
    end
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; positions(50) - xf(1); positions(end) - xf(2); vels(50); vels(end)];
end