classdef ArmModel
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        I1 = 0.01481;
        m1 = 1.864572;
        I2 = 0.019281;
        m2 = 1.534315;
        lc1 = 0.180496;
        lc2 = 0.181479;
        l1 = 0.26;
        l2 = 0.36;
        g = 9.806;
        dt = 0.01;
    end
    
    methods
        
        function obj = ArmModel()
            obj.I1 = obj.I1 + obj.m1*obj.lc1^2;
            obj.I2 = obj.I2 + obj.m2*obj.lc2^2;
        end
        
        function xdd = forward(obj, x, u)
            q = x(1:2,1);
            qd = x(3:4,1);
            
            temp = obj.m2*obj.l1*obj.lc2*cos(q(2));
            H = [obj.I1 + obj.I2 + obj.m2*obj.l1^2 + 2*temp, obj.I2 + temp;
                 obj.I2 + temp, obj.I2];
            
            temp = obj.m2*obj.l1*obj.lc2*sin(q(2));
            C = [-2*temp*qd(2), -temp*qd(2);
                 temp*qd(1), 0];
             
            G = [(obj.m1*obj.lc1 + obj.m2*obj.l1)*obj.g*sin(q(1)) + obj.m2*obj.g*obj.l2*sin(q(1)+q(2));
                 obj.m2*obj.g*obj.l2*sin(q(1)+q(2))];
            
            B = eye(2);
            
            xdd = inv(H)*(B*u - C*qd - G);
            
%             xdd = zeros(2,1);
%             xdd(1,1) = u(1);
%             xdd(2,1) = u(2);
        end
        
        function [A, B] = linearize(obj, x, u)
            
            A = eye(4);
            A(1,3) = obj.dt;
            A(2,4) = obj.dt;
            B = zeros(4, 2);
            
            for i = 1:1:4
                xp = x;
                xp(i,1) = xp(i,1) + 0.01;
                xddp = obj.forward(xp, u);
                xn = x;
                xn(i,1) = xn(i,1) - 0.01;
                xddn = obj.forward(xn, u);
                delta = (xddp - xddn)./0.02;
                A(3:4,i) = delta.*obj.dt;
            end
            
            for i = 1:1:2
                up = u;
                up(i,1) = up(i,1) + 0.01;
                xddp = obj.forward(x, up);
                un = u;
                un(i,1) = un(i,1) - 0.01;
                xddn = obj.forward(x, un);
                delta = (xddp - xddn)./0.02;
                B(3:4,i) = delta.*obj.dt;
            end
        end
        
        function [x_traj, u_traj] = run_forward(obj, x0, x_traj, u_traj, K, k, use_feedback)
           
            steps = length(u_traj);
            
            x = x0;
            for i = 1:1:steps
               u = (u_traj(i,:)') + k{i};
               if (use_feedback)
                   xe = x - x_traj(i,:)';
                   u = u + K{i}*xe;
               end
               xdd = obj.forward(x, u);
               x(3) = x(3) + xdd(1)*obj.dt;
               x(4) = x(4) + xdd(2)*obj.dt;
               x(1) = x(1) + x(3)*obj.dt;
               x(2) = x(2) + x(4)*obj.dt;
               
               x_traj(i,:) = x';
               u_traj(i,:) = u';
            end
            
        end
        
        function playback(obj, x_traj)
        
            for i = 1:1:length(x_traj)
               
                x = x_traj(i,:)';
                
                p1 = [0, 0];
                p2 = [p1(1) + obj.l1*sin(x(1)); p1(2) - obj.l1*cos(x(1))];
                p3 = [p2(1) + obj.l2*sin(x(1)+x(2)); p2(2) - obj.l2*cos(x(1)+x(2))];
                
                plot([p1(1), p2(1)], [p1(2), p2(2)], 'k', 'LineWidth', 5)
                hold on
                plot([p2(1), p3(1)], [p2(2), p3(2)], 'k', 'LineWidth', 5)
                xlim([-1 1])
                ylim([-1 1])
                hold off
                pause(0.001);
                
            end
            
        end
    end
    
end

