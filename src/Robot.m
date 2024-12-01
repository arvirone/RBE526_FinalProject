% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        mDHTable;
        dhSym;
        goalPosition;
        model;
        dataRecoder;
        jacobSym; % Symbolic Table
        s; % Theta variables
        jacob;   
        round;
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            self.round=0;
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);

            % Robot Dimensions
            self.mDim = [96.326, sqrt(128^2+24^2), 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            self.mDHTable = [0 self.mDim(1) 0 -90;...
                            (atand(self.mOtherDim(2)/self.mOtherDim(1)) - 90) 0 self.mDim(2) 0;...
                            (90 - atand(self.mOtherDim(2)/self.mOtherDim(1))) 0 self.mDim(3) 0;...
                           0 0 self.mDim(4) 0];
            
            self.model = Model(self);
            self.dataRecoder = DataRecorder(self);
            % Symbolic
            syms theta1 theta2 theta3 theta4   
            self.dhSym = [theta1 self.mDim(1) 0 -90;...
                    (atand(self.mOtherDim(2)/self.mOtherDim(1)) - 90)+theta2 0 self.mDim(2) 0;...
                     (90 - atand(self.mOtherDim(2)/self.mOtherDim(1)))+theta3 0 self.mDim(3) 0;...
                     theta4 0 self.mDim(4) 0];
           self.s = [theta1; theta2; theta3; theta4];
           T = self.dh2fk(self.dhSym,dh2mat(self,[0 0 0 0]));
           self.jacobSym = sym('a',[6   4]);
           for i = 1:4
               self.jacobSym(1,i) = diff(T(1,4),self.s(i));
               self.jacobSym(2,i) = diff(T(2,4),self.s(i));
               self.jacobSym(3,i) = diff(T(3,4),self.s(i));
           end            
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
            self.goalPosition = goals;
            self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            %disp("time")
            %disp(time_ms)
            %disp("acc time")
            %disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end
        
        % Gets the current joint positions and velocities
        % readings [2x4 double] - the joints' positions and velocities
        % (deg, deg/s)
        % rows will be 0 if its flag is false
        function readings = measured_js(self, GETPOS,GETVEL)
            readings = zeros(2,4);
            if(GETPOS)
                readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end
            if(GETVEL)
                readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
                readings(2,2) = readings(2,2)-65.952+4.211;
                readings(2,3) = readings(2,3)-65.952;
            end
            return;
        end
        % returns goal position
        % goal [1x4 double] Goal Positions
        function goal = goal_js(self)
            goal=self.goalPosition;
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end
        % Adds interpolation for joint movements
        % degrees [1x4] - angles in degrees
        % time double - time to complete move in seconds
        function interpolate_jp(self,degrees, time)
            self.writeTime(time); % Write travel time
            self.writeJoints(degrees)
            self.goalPosition=degrees;
        end    
        % Sends the joints a desired angle in degrees
        % degrees [1x4 double] - angles in degrees
        function servo_jp(self, degrees)
            self.writeTime(0.3);
            self.writeJoints(degrees)
            self.goalPosition=degrees;
        end
        
        % Returns [1x4] of the current set point of all joints
        function readings = setpoint_js(self)
            readings = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
        end
        % Solves FK for a given joint space
        % Takes a 1x4 matrix  (row of DH table)
        % returns a 4x4 homogeneous transformation matrix to go from the
        % base frame to the end effector frame
        function htm  = dh2mat(self,dhRow)
            theta = dhRow(1);
            d=dhRow(2);
            a=dhRow(3);
            alpha = dhRow(4);
                htm=[cosd(theta), -sind(theta)*cosd(alpha), sind(theta)*sind(alpha),a*cosd(theta);
                    sind(theta), cosd(theta)*cosd(alpha), -cosd(theta)*sind(alpha), a*sind(theta);
                    0,sind(alpha),cosd(alpha),d;
                    0,0,0,1];
        end
        
        % Takes an nx4 array corresponding to n rows of a DH table
        % Generates a 4x4 Symbolic matrix by multiplying all the T
        % matricies together
        function shtm = dh2fk(self, dhTable,t01)
            shtm = t01;
            for i = 1:1:width(dhTable)
                shtm = shtm*dh2mat(self,dhTable(i,:));
            end
            return;
        end

        % Takes an nx1 vector (joint angles)
        % utilizes dh2fk
        % Returns a 4x4 matrix representing the end effector tip position
        % and orientation with respect to the base frame
        function fk = fk3001(self, jointAngles)
            dhTable = self.mDHTable;
            % substitute join varuables into dhtable 
            for i=1:1:width(jointAngles)
                dhTable(i,1) = dhTable(i,1)+jointAngles(i);
            end
            % find homogeonious matrix with dhtable containing joint angles
            fk = dh2fk(self,dhTable,dh2mat(self,[0 0 0 0]));
        end

        % Returns a 4x4 homogenius transformation matrix based on the
        % current joint positions
        function fk = measured_cp(self)
            js = measured_js(self,1,0);
            fk = fk3001(self, js(1,:));
            return;
        end

        % Returns a 4x4 homogenius transformation matrix based on the
        % current joint setpoint
        function fk = setpoint_cp(self)
            fk = fk3001(self, setpoint_js(self));
            return;
        end

        % Returns a 4x4 homogenius transformation matrix based on the
        % joint goal positions
        function fk = goal_cp(self)
            fk = fk3001(self, goal_js(self));
            return;
        end
        % Takes a 4x1 task space vector 
        % Returns a 4x1 joint space vector 
        function js= ik3001(self,point)
            js=[0 0 0 0];
            X0=point(1);
            Y0=point(2);
            Z0=point(3);
            alpha = point(4);
            % Theta 1, add an if statement if y is less than 0 
            r0 = sqrt(X0^2+Y0^2);
            js(1) = atan2d(sqrt(1-(X0/r0)^2),X0/r0);

            if Y0<0
                js(1)=-js(1);
            end
            if Y0==0 && X0==0
                js(1)=0;
            end
            % EE
            ree= self.mDim(4)*cosd(alpha);
            Zee = self.mDim(4)*sind(alpha);
            Xee = ree*cosd(js(1));
            Yee=ree*sind(js(1));
            % EE Offset
            Xc=X0-Xee;
            Yc=Y0-Yee;
            Zc=Z0+Zee;
            r1 = sqrt(Xc^2+Yc^2);
            % Wrist Axis opposite
            if r0<self.mDim(4)
                r1 = -r1;
            end
            
            % Theta 3
            s=Zc-self.mDim(1);
            D1=-1*(self.mDim(2)^2+self.mDim(3)^2-(r1^2+s^2))/(2*self.mDim(2)*self.mDim(3));
            js(3)=atan2d(sqrt(1-D1^2),D1);
            % Theta 2
            D2 = (self.mDim(2)^2 + r1^2 + s^2- self.mDim(3)^2)/(2*self.mDim(2)*sqrt(r1^2+s^2));
            beta = atan2d(sqrt(1-D2^2),D2);
            H = r1/sqrt(r1^2+s^2);
            a=atan2d(-1*sqrt(1-H^2),H); 
            if(Zc>=self.mDim(1))
                js(2)=a-beta;
                %disp(a);
            else
                js(2)=-a-beta;
            end
            % Theta 4
            js(4) = alpha-js(3)-js(2);
            % Offset theta 2,3
            js(2) = js(2)+79.380345;
            js(3) = js(3)-79.380345;
            
        end

        %Takes a 4xn set of coefficients, total time and boolean task
        % Runs the given trajectory
        function run_trajectory(self, coeffs, totalTime, task)
            tic;
            m = zeros(1,4);
            self.round = self.round +1;
           while totalTime > toc
               t = toc;
               
               for i=1:4
                       %Logic to determine if cubic or quintic
                       if width(coeffs) ==6 
                       m(1,i) = coeffs(1,i)+ coeffs(2,i)*t+ coeffs(3,i)*t^2+coeffs(4,i)*t^3+coeffs(5,i)*t^4+coeffs(6,i)*t^5;
                       else
                       m(1,i) = coeffs(1,i)+ coeffs(2,i)*t+ coeffs(3,i)*t^2+coeffs(4,i)*t^3;
                       end 
               end
               %Plot Arm
                q = self.measured_js(1,1);
                self.model.plot_arm(q(1,1:4),q(2,1:4));
                determ = det(self.jacob(1:3,1:3));
%                 self.dataRecoder.plot_ee();
% fdk = self.fdk3001(q(1,:),q(2,:));
%                 time = totalTime*self.round+toc;
%                 writematrix([time sqrt(fdk(3,1)^2+fdk(2,1)^2+fdk(3,1)^2)],'MData.csv', 'WriteMode','append');
               if(determ > 5)
               %Logic to run in JS or TS
                if task == false
                    self.servo_jp(m);  %JS
                else
                    self.servo_jp(self.ik3001(m));%TS
                end
                %pause(.015);
               else
                   self.model.eStop();
                   curPos = self.measured_js(1,0);
                   self.servo_jp(curPos(1,:));  %JS
                   disp("not reachable");
                   break;
               end
           end

        end

       %Take a 4x1 set of joint angles
       %Returns a 6x4 Jacobian Matrix
       function jacob = jacob3001(self,q)
           syms theta1 theta2 theta3 theta4  
           self.jacob = zeros(6,4);
           for i = 1:length(q)
               self.jacob(1,i) = subs(self.jacobSym(1,i),self.s,q);
               self.jacob(2,i) = subs(self.jacobSym(2,i),self.s,q);
               self.jacob(3,i) = subs(self.jacobSym(3,i),self.s,q);
               jacob = self.jacob;
           end 
               dhTable = self.mDHTable;
              for i=1:1:length(q)
                dhTable(i,1) = dhTable(i,1)+q(i);
              end
              t01 = dh2mat(self,dhTable(1,:));
              t02 = t01* dh2mat(self,dhTable(2,:));
              t03 = t02* dh2mat(self,dhTable(3,:));
              t04 = t03 *dh2mat(self,dhTable(4,:));
              m1 = t01(:,3);
              m1(4,:) =[];
              m2 = t02(:,3);
              m2(4,:) =[];
              m3 = t03(:,3);
              m3(4,:)=[];
              m4= t04(:,3);
              m4(4,:) = [];

              jacob(4:end,:) = horzcat(m1,m2,m3,m4);
  

       end
       % Calculates forward velocity kinematics
       % Takes q: Current Joint Angles
       % Takes qDot: Current Joint Velocities
       % Returns: 6x1 task space vector with linear and angular velocities 
       function pDot = fdk3001(self, q, qDot)
           q = transpose(q);
           qDot = transpose(qDot);
            j = self.jacob3001(q);
            pDot = j*qDot;
       end
       %Calculates the positions of a point in checkerboard space to task space
       % Takes a 1x2 cordinate in checkerboard frame
       % returns 1x3 coordinate in task plane [x,y,z]
       function cord = boardToRobot(self, c)
           Tbasetoboard = [0 1 0 95;
                1 0 0 -110;
                0 0 -1 25;
                0 0 0 1];
            pointC = [1 0 0 c(1);
                0 1 0 c(2);
                0 0 1 0;
                0 0 0 1];
            pointTs = Tbasetoboard*pointC;
%            cord = [c(2)+95 c(1)-105 25];
            cord = [pointTs(1,4),pointTs(2,4),pointTs(3,4)];
       end
    end % end methods
end % end class 
