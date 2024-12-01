classdef Model
properties
figure
robot
end
methods
    function obj = Model(robot)
        obj.robot = robot;
           obj.figure = plot3(0,0,0);
           grid on
    end
    %Plots the arms 3d stick plot
    % q : Joint Values

    function eStop (obj)
%         title("E Stop :(")
    end

    function plot_arm(obj, q, v)
        DHT = obj.robot.mDHTable;
        for i=1:1:width(q)
                DHT(i,1) = DHT(i,1)+q(i);
        end
        T00 = obj.robot.dh2mat([0 0 0 0]);
        T01 = obj.robot.dh2mat(DHT(1,:));
        T02 = T01 * obj.robot.dh2mat(DHT(2,:));
        T03 = T02 * obj.robot.dh2mat(DHT(3,:));
        T04 = T03 * obj.robot.dh2mat(DHT(4,:));
        trans= [T00 T01 T02 T03 T04];

        X = [0 T01(1,4) T02(1,4) T03(1,4) T04(1,4)];
        Y = [0 T01(2,4) T02(2,4) T03(2,4) T04(2,4)];
        Z = [0 T01(3,4) T02(3,4) T03(3,4) T04(3,4)];
       
        for i=1:1:5 % Joints
            a=4*(i-1)+1;
            b=4*(i);
            T = trans(:,a:b);
            for j=1:1:3 % Axis
                if i==1&&j==2 %anbles hold after the first call
                    hold on
                end
         quiver3(X(i),Y(i),Z(i),T(1,j),T(2,j),T(3,j),35,'linewidth',2);
            end
        end


       vel_ts = obj.robot.fdk3001(q,v);
          %      display(vel_ts);
       quiver3(X(5),Y(5),Z(5),vel_ts(1),vel_ts(2),vel_ts(3),.75,'linewidth',2);
        xlim([-500 500]);
        ylim([-500,500]);
        zlim([0 500]);
        xlabel("X (mm)");
        ylabel("Y (mm)")
        zlabel("Z (mm)");
        plot3(X,Y,Z,'linewidth',2, 'Color','black')
        pbaspect([2 2 1])
        hold off
        shg
    end
end
end