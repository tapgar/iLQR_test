classdef iLQR
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m_K
        m_k
        Qf = 10.*diag([10.0, 10.0, 2.0, 2.0]);
        R = diag([0.01, 0.01]);
        arm
        iters
        x_targ
        last_cost
        lambda
    end
    
    methods
        function obj = iLQR(iters, target_x)
           obj.arm = ArmModel();
           obj.m_K = {};
           obj.m_k = {};
           obj.iters = iters;
           for i = 1:1:iters
               obj.m_K{i} = zeros(2,4);
               obj.m_k{i} = zeros(2,1);
           end
           obj.x_targ = target_x;
           obj.last_cost = 10000000.0;
           obj.lambda = 1.0;
        end
        
        function RuniLQR(obj, x0)
           
            while (1)
               
                [x_traj, u_traj, x_open, u_open] = obj.RunForward(x0);
                obj = obj.RunBackward(x0, x_traj, u_traj, x_open, u_open);
%                 obj.arm.playback(x_traj);
%                 pause(0.01)
            end
            
        end
        
        function [x_traj, u_traj, x_open, u_open] = RunForward(obj, x0)
            x_traj = zeros(obj.iters, 4);
            u_traj = zeros(obj.iters, 2);
            [x_open, u_open] = obj.arm.run_forward(x0, x_traj, u_traj, obj.m_K, obj.m_k, false);
            [x_traj, u_traj] = obj.arm.run_forward(x0, x_open, u_traj, obj.m_K, obj.m_k, true);
        end
        
        function obj = RunBackward(obj, x0, x_traj, u_traj, x_open, u_open)
           
            prev_x_traj = x_traj;
            prev_u_traj = u_traj;
            prev_K = obj.m_K;
            prev_k = obj.m_k;
            
            bNotDecreasing = true;
            
            q_hist = {};
            for i = 1:1:obj.iters
                q_hist{i} = struct('Qu',0,'Quu',0,'Qux',0);
            end
            
            lx = zeros(4,1);
            lxx = zeros(4,4);
            lux = zeros(2,4);
            
            while (bNotDecreasing)
                
                Qu = obj.R*(u_traj(obj.iters,:)');
                Quu = obj.R;
                Qx = 2.0.*obj.Qf*(x_traj(obj.iters,:)' - obj.x_targ);
                Qxx = 2.0.*obj.Qf;
                Qux = zeros(2,4);
                
                Vx = Qx;
                Vxx = Qxx;
                
                q_hist{obj.iters}.Qu = Qu;
                q_hist{obj.iters}.Qux = Qux;
                q_hist{obj.iters}.Quu = Quu;
                
                for i = obj.iters-1:-1:1
                    u = u_traj(i,:)';
                    lu = 2.*obj.R*u;
                    luu = 2.*obj.R;
                    
                    [A, B] = obj.arm.linearize(x_traj(i,:)', u);
                    
                    Qx = lx + A'*Vx;
                    Qu = lu + B'*Vx;
                    Qxx = lxx + A'*Vxx*A;
                    Qux = lux + B'*Vxx*A;
                    Quu = luu + B'*Vxx*B;
                    
                    [U, S, V] = svd(Quu);
                    S(S < 0) = 0.0;
                    S = S + eye(2).*obj.lambda;
                    Sd = inv(diag([S(1,1), S(2,2)]));
                    
                    Quu_inv = U*Sd*V';
                    
                    obj.m_K{i} = -Quu_inv*Qux;
                    obj.m_k{i} = obj.m_k{i} -Quu_inv*Qu;
                    
                    Vx = Qx - obj.m_K{i}'*Quu*obj.m_k{i};
                    Vxx = Qxx - obj.m_K{i}'*Quu*obj.m_K{i};
                    
                    q_hist{i}.Qu = Qu;
                    q_hist{i}.Qux = Qux;
                    q_hist{i}.Quu = Quu;
                end
                
                [x_check, u_check] = obj.arm.run_forward(x0, x_open, u_open, obj.m_K, obj.m_k, false);
                
                cost = obj.CalculateTrajCost(x_check, u_check);
                
                display(sprintf('Cost: %f | Last Cost: %f\n', cost, obj.last_cost));
                
                if (cost < obj.last_cost)
                    obj.lambda = obj.lambda / 1.01;
                    bNotDecreasing = false;
                    obj.last_cost = cost;
                else
                    obj.lambda = obj.lambda * 1.01;
                    obj.m_k = prev_k;
                    obj.m_K = prev_K;
                    x_traj = prev_x_traj;
                    u_traj = prev_u_traj;
                end
                
            end
            
        end
        
        function cost = CalculateTrajCost(obj, x_traj, u_traj)
            cost = (obj.x_targ' - x_traj(end,:))*obj.Qf*(obj.x_targ - x_traj(end,:)');
            for i = 1:1:obj.iters
               cost = cost + u_traj(i,:)*obj.R*u_traj(i,:)'; 
            end
        end
        
    end
    
end

