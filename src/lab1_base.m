%% Setup robot   
travelTime = ; % Defines the travel time
robot = Robot(); % Creates robot object
model = Model(robot);
planner = Traj_Planner();
% dataRecorder = DataRecorder(robot);
% camera = Camera();
robot.writeMotorState(true); % Write position mode
%% Program 
%robot.writeGripper(false); 
robot.interpolate_jp([270 30 -30 90],travelTime); 
% camera.getCameraPose() %% T image to t checker (0,0)
% USE THESE THREE LINES
%imagePixel = [622 375];
% checkerCords = pointsToWorld(camera.cam_IS,camera.getRotationMatrix,camera.getTranslationVector,imagePixel);
%pointTs = robot.boardToRobot(checkerCords);
%robot.interpolate_jp(robot.ik3001([pointTs(1) pointTs(2) pointTs(3) 90]),2);
%pause(10);
%robot.interpolate_jp([-90 -90 80 20],2);
%pause(5);
% camera.getObjectPos();
% display(checkerCords);
% display(pointTs);  

% RGBVals =   [241, 195, 109;
%              170, 196, 198;
%              239, 236, 167;
%              81, 199, 147;
%              217, 115, 97];
% RGBThresh = 10;
% Colors = ["Orange"; "Gray"; "Yellow"; "Green"; "Red"];
% ColorCounter = 0;
% 
% im = imread('pic.png');
% I = rgb2hsv(im);
% 
% I = imsharpen(I);
% 
% % Define thresholds for channel 1 based on histogram settings
% channel1Min = 0.000;
% channel1Max = 1.000;
% 
% % Define thresholds for channel 2 based on histogram settings
% channel2Min = 0.128;
% channel2Max = 1.000;
% 
% % Define thresholds for channel 3 based on histogram settings
% channel3Min = 0.559;
% channel3Max = 1.000;
%             
% % Create mask based on chosen histogram thresholds
% sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
% (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
% (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
% BW = sliderBW;
% 
% % Initialize output masked image based on input image.
% maskedRGBImage = im;
% 
% % Set background pixels where BW is false to zero.
% maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
% 
% sat = maskedRGBImage(:,:,2);
% imBW = (sat> graythresh(sat));
% med =medfilt2(imBW);
% e = edge(med,"canny");
% f = imfill(e,4,'holes');
% 
% labeledImage = bwlabel(f);
% stats = regionprops(labeledImage, 'Area');
% for i = 1:numel(stats)
%     if stats(i).Area < 1400
%         labeledImage(labeledImage == i) = 0;
%     end
% end
% filteredBinaryImage = labeledImage> 0;
% s = regionprops(filteredBinaryImage,'basic');
% imshow(im);
% hold on
% centroids =cat(1,s.Centroid);
% scatter(centroids(:,1),centroids(:,2),300,"Marker", "+","Color","black");
% hold off
% for i = 1:height(centroids)
%     for j=1:5
%         PixelColor = im(round(centroids(i,2)), round(centroids(i,1)),:);
%         ColorCounter =0;
%         for k=1:3
%             if(PixelColor(:,:,k) <=(RGBVals(j,k)+RGBThresh))&&( PixelColor(:,:,k)>=(RGBVals(j,k)-RGBThresh))
%                 ColorCounter= ColorCounter+1;
%             end
%         end
%         if ColorCounter ==3
%             Color = Colors(j,:);
%         end
%     end
% text(centroids(i,1),centroids(i,2)-35,Color + " " + num2str(centroids(i,1)+ " " + num2str(centroids(i,2))),"FontSize",20,"HorizontalAlignment","center","FontWeight","bold","Color",im(round(centroids(i,2)), round(centroids(i,1)),:));
% end