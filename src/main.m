%% Setup robot   
travelTime = 1; % Defines the travel time
robot = Robot(); % Creates robot object
model = Model(robot);
% camera = Camera();
% planner = Traj_Planner();
% dataRecorder = DataRecorder(robot);
robot.writeMotorState(true); % Write position mode
%% Program 
%robot.writeGripper(false); 
robot.interpolate_jp([270 30 -30 90],travelTime); 
joint1 = 270;
frameSkip = 500;  
frameCount = 0;

while(true)
frameCount = frameCount + 1;
    
    
    if mod(frameCount, frameSkip) == 0

raw_img =  snapshot(webcam(2));
% imshow(raw_img);

I= rgb2hsv(raw_img);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.537;
channel1Max = 0.653;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.479;
channel2Max = 0.840;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.713;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = bwareaopen(sliderBW, 10000);

% Initialize output masked image based on input image.
maskedRGBImage = raw_img;
% rotatedImg = imrotate(raw_img, -joint1+270);

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
props = regionprops(BW,'BoundingBox','Area','Centroid');
imshow(imrotate(raw_img,180));  % Show the original RGB image
centroidsX=[];
centroidsY = [];
hold on;
for k = 1:length(props)
    if(props(k).Area > 10000)
%           rectangle('Position', props(k).BoundingBox, 'EdgeColor', 'r', 'LineWidth', 2);
%           plot(props(k).Centroid(1),props(k).Centroid(2),'ro', 'MarkerFaceColor', 'r')
        centroidsX = [centroidsX ,props(k).Centroid(1)];
        centroidsY = [centroidsY ,props(k).Centroid(2)];
    end
end

         if(mean(centroidsX) < 720)
             joint1 = joint1+2;
         end
         if(mean(centroidsX)  > 1200)
             joint1 = joint1-2;
         end
         if(mean(centroidsY)+50 < 405)
             joint4 = joint4-.5;
         end
         if(mean(centroidsY)+50  > 675)
             joint4 = joint4+.5;
         end
hold off;
if(joint1>200 && joint1<340)
    robot.interpolate_jp([joint1 30 -30 joint4],travelTime); 
end
    end
end