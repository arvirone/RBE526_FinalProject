classdef DataRecorder
properties
figure
robot
data
end
methods
    
    function obj = DataRecorder(robot)
        obj.robot = robot;
           obj.figure = plot(0,0);
           obj.data = zeros(1,4);
           %grid on
    end
    function plotDataLV(obj)
        plot(obj.data);
    end
    function logData(obj,data)
        obj.data(height(data),:) = data;
    end
    %Plots the ee position, doesnt erase old points 
    function plot_ee(obj)
        T=obj.robot.measured_cp();
        scatter3(T(1,4),T(2,4),T(3,4),Marker=".");
        T=obj.robot.goal_cp();
%         scatter3(T(1,4),T(2,4),T(3,4),'linewidth',6);
        xlim([-500 500]);
        ylim([-500,500]);
        zlim([0 500]);
        xlabel("X (mm)");
        ylabel("Y (mm)")
        zlabel("Z (mm)");
        title("End Effector Position");
        hold on;
        shg
    end
    % Plots X,Y,Z with respect to T, (2D Graph)
    % toc : Time (s)
    function plot_ts(obj,toc)
        point = obj.robot.measured_cp();
        plot(toc,point(1,4),'color','red',MarkerSize=3,Marker='.');
        plot(toc,point(2,4),'color','green',MarkerSize=3,Marker='.');
        plot(toc,point(3,4),'color','blue',MarkerSize=3,Marker='.');

        legend('X','Y','Z');
        xlabel("Time (s)");
        ylabel("Position (mm)");
        title("Triangle Move");
        hold on
        shg
    end
    % Plots joint angles with respect to T, (2D Graph)
    % toc : Time (s)
    function plot_js(obj,toc)
        point = obj.robot.measured_js(true,false);
        plot(toc,point(1,1),'color','black',MarkerSize=3,Marker='.');
        plot(toc,point(1,2),'color','red',MarkerSize=3,Marker='.');
        plot(toc,point(1,3),'color','green',MarkerSize=3,Marker='.');
        plot(toc,point(1,4),'color','blue',MarkerSize=3,Marker='.');
        legend('T1','T2','T3','T4');
        xlabel("Time (s)");
        ylabel("Position (deg)");
        title("Triangle Move");
        hold on
        shg
    end
end
end