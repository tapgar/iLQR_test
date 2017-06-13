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
        l1 = 0.36;
        l2 = 0.46;
        g = 9.806;
    end
    
    methods
        
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
                 obj.m2*obj.g*sin(q(1)+q(2))];
            
            B = ones(2,1);
            
            xdd = (B*u - C*qd - G)/H;
        end
        
        function [A, B] = linearize(obj, x, u)
            
        end
        
        
    end
    
end

