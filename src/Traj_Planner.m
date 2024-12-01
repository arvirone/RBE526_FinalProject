classdef Traj_Planner
    %TRAJ_PLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = Traj_Planner()
      
        end
        
        % cubic_traj - Cubic trajectory between two via-points
        % t0 : Starting time(seconds)
        % tf : End time (seconds)
        % q0 : Starting Pos
        % qf : End Pos
        % v0 : Startin velocity
        % vf : End Velocity
        % Output : 1x4 array of coeffeicents 
        function output = cubic_traj(obj, t0, tf, q0, qf, v0, vf)            
            M = [1, t0, t0^2, t0^3;
                0, 1, 2*t0, 3*t0^2;
                1, tf, tf^2, tf^3;
                0, 1, 2*tf, 3*tf^2];
            B = [q0; v0; qf; vf];
            output = inv(M)*B;
        end


   % quintic_traj - Quintic trajectory between two via-points
        % t0 : Starting time(seconds)
        % tf : End time (seconds)
        % q0 : Starting Pos
        % qf : End Pos
        % v0 : Startin velocity
        % vf : End Velocity
        % a0 : Starting acceleration
        % af : End accleration
        % Output : 1x6 array of coeffs
        function output = quintic_traj(obj, t0, tf, q0, qf, v0, vf, a0, af)
          M = [1, t0, t0^2, t0^3, t0^4, t0^5;
                0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
                0, 0, 2, 6*t0, 12*t0^3, 20 * t0^3;
                1, tf, tf^2, tf^3, tf^4, tf^5;
                0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
                0, 0, 2, 6*tf, 12*tf^2, 20*tf^3
                ];
            B = [q0; v0; a0; qf; vf; af];
            output = inv(M)*B;
        end

    end
end

