joints = readmatrix('MData.csv');
hold on
plot(joints(:,1),joints(:,2),'b');
% plot(joints(:,1),joints(:,3),'r');
% plot(joints(:,1),joints(:,4),'g');
legend('Velocity Magnatude');
xlabel('Time (Seconds)');
ylabel('velocity (mm/s)');
title('Joint Space Velocity Mag Plot')
hold off
xlim([3 12.5]);

% hold on 
% plot(xz(:,1), xz(:,2),'b');
% plot(xz(:,1),xz(:,4),'r');
% legend('X Axis','Z Axis');
% xlabel('Time (Seconds)');
% ylabel('End Effector Position (mm)');
% title('X-Z End Effector Position');
% hold off
% 
% 
% hold on
% 
% plot(xz(:,2),xz(:,4));
% plot(219.554,54.8353, 'o', 'Color','blue');
% plot(282.611,214.953, 'o', 'Color', 'black');
% plot(-146.857,433.313,'o','Color', 'red')
% xlabel('End Effector X Axis (mm)');
% ylabel('End Effector Z Axis (mm)');
% title('X-Z End Effector Position');
% 
% hold off
% 
% 
% 
% hold on
% 
% plot(xz(:,2),xz(:,3));
% xlabel('End Effector X Axis (mm)');
% ylabel('End Effector Y Axis (mm)');
% title('X-Y End Effector Position');
% hold off