% KITTI RAW DATA DEVELOPMENT KIT
% 
% Demonstrates projection of the velodyne points into the image plane

% clear and close everything
clear all; close all; dbstop error; clc;
disp('======= KITTI DevKit Demo =======');
%{
% options (modify this to select your sequence)
base_dir  = 'C:\Users\Administrator\Documents\KITTI\2011_09_26_drive_0052';
calib_dir = 'C:\Users\Administrator\Documents\KITTI\2011_09_26_calib';
cam       = 2; % 0-based index
frame     = 2; % 0-based index

% load calibration
calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
Tr_velo_to_cam = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'));

% get number of images for this dataset
nimages = length(dir(fullfile(sprintf('%s/image_%02d/data',base_dir,cam,frame), '*.png')));

% fprintf('test');
% compute projection matrix velodyne->image plane
R_cam_to_rect = eye(4);
R_cam_to_rect(1:3,1:3) = calib.R_rect{1};
P_velo_to_img = calib.P_rect{cam+1}*R_cam_to_rect*Tr_velo_to_cam;

% load and display image
img = imread(sprintf('%s/image_%02d/data/%010d.png',base_dir,cam,frame));
fig = figure('Position',[20 100 size(img,2) size(img,1)]); axes('Position',[0 0 1 1]);
% figure;
% while(1)
% img = imread(sprintf('%s/image_%02d/data/%010d.png',base_dir,cam,frame));
img = imread(sprintf('%s/image_%02d/data/%010d.png',base_dir,cam,frame));
imshow(img); hold on;

% load velodyne points
fid = fopen(sprintf('%s/velodyne_points/data/%010d.bin',base_dir,frame),'rb');
velo = fread(fid,[4 inf],'single')';
velo= velo(1:10:end,:); % remove every 5th point for display speed
fclose(fid);

% remove all points behind image plane (approximation
velo1 = velo;

idx = velo(:,1)<0;
velo(idx,:) = [];
idz = velo(:,3)<-2;
velo(idz,:) = [];


% project to image plane (exclude luminance)
velo_img = project(velo(:,1:3),P_velo_to_img);

% plot points
cols = jet;%

% for i=1:size(velo_img,1)
%   col_idx = round(64*5/velo1(i,1));
%   plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',4,'MarkerSize',2,'Color',cols(col_idx,:));
% end

for i = 1:size(velo_img, 1)
    col_idx = round(64*5/velo1(i,1));
end
% figure(1);
plot(velo_img(:,1), velo_img(:,2),'.');
% fig2 = figure;
% hold off;
% drawnow;
figure(2);
plot3(velo(:,2),velo(:,1),velo(:,3),'r.');
axis equal;
% set(fig2, 'xData', velo(:,1), 'yData', velo(:,2), 'zData', velo(:,3)); 
drawnow;
fprintf('frame %d\n', frame);
% velo2 = flipud(velo);
% velo1 = flipud(velo);
count = 1;
for i = 2:size(velo, 1)
    if velo1(i-1,2) > 0 && velo1(i,2) < 0
        velo2(i,2) = 1;
%         fprintf('%d points\n', count);
        count = 0;
    else
        velo2(i,2) = 0;
        count = count + 1;
    end
end

% figure(3);
% plot(velo1(:,1),'ro-');
% figure(4);
% plot(velo1(:,2),'rs-');
% hold on;
% plot(velo2(:,2),'bs');
figure(5);
plot(velo1(1:120,2), velo1(1:120,1),'rs-');

% figure(6);
% [nx, ny, nz] = surfnorm(velo(:,1:3));
% % mesh(velo(:,1:2));
% quiver3(velo(:,2),velo(:,1),velo(:,3),nx(:,3),ny(:,3),nz(:,3));
% hold on
% plot3(velo(:,2),velo(:,1),velo(:,3),'r.');
% % surf(velo(:,2),velo(:,1),velo(:,3));
% colormap hsv
% view(-35,45)
xlabel('Y value');
ylabel('X value');
frame = frame + 1;
% display(frame);
if (frame+2) == nimages
    fprintf('%d frame\n', frame);
    break;
end
p.sensortype = 1; % 1: LRF
                  % 2: TOF
p.fastmode = 1; % 1: without region growing
                % 0.5: simple region growing: under consideration
                % 0: point-based region growing: under consideration
p.RPCA = 1; % 1: with Robust PCA
            % 0: without Robust PCA
        % LRF type       
        % Extraction parameter for Timetest data
        p.threError = 0.01; % threshold for function 'CheckPlane' 
        p.threAng = 5; % threshold for similarlity of normal vector
        p.threDist = 0.02; % threshold for similarlity of distance
        p.nOutlier = 200; % the minimum number of points composed one plane
        p.nOutsize = 0.050; % the smallest length of a plane for function 'pruning'
        p.nOutsize2 = 0.05; % the smallest length of a plane for function 'pruning2'
        p.smallLength = 0.05; % the smallest length of a cube
        p.nRPCAiter = 30; % the maximum number of iteration of RPCA
        p.RPCAweight = 0.5; % the rate of fully weighted point for RPCA
        p.RPCAweightbound = 10; % the maximum MDist to decide weight for RPCA
%% Line 3. Initializing potential queue and plane queue
disp('initializing...');
cloud = velo(:,1:3);
potentialqueue = []; % the queue that saves potentials
planequeue = []; % the queue that saves intact planes
remainderqueue =[]; % the queue that saves small fragments

plane.index=1:size(cloud,1);
plane.level=1;

%% Line 4. First segmentation and filling potential queue
disp('segmenting...');

[plane, p] = firstsegmentation(cloud,plane,p);
potentialqueue = [potentialqueue plane];

plot_test_HM2(cloud, potentialqueue); % display the result after the first segmentation

%% Line 5. while (potential queue is not empty)
%{
while ~isempty(potentialqueue) % repeat while loop until potential queue is empty
    interestplane =[];
    interestplane = potentialqueue(1); % take out the information and point index from potentialqueue, and put them into interestplane
    potentialqueue = potentialqueue(2:end);

    % Line (5a). For each cube, calculate plane parameters
    [interestplane.eq,interestplane.center] = Extraction(cloud, interestplane.index, p);
    
    % Line (5b)~(5c). 
    isPlane = CheckPlane(cloud, interestplane, p); % check whether the points compose one plane or not

    
    if  isPlane == 0 % 5c. if variance is large, segment and push data to plane queue
        plane = segmentation(cloud,interestplane,p); % divide a cube into small eight cubes
        if ~isempty(plane)
            potentialqueue = [potentialqueue plane];
        else
            remainderqueue = [remainderqueue interestplane];
        end        
    else % 5b. if variance is small enough, push data to potential queue
        planequeue = [planequeue interestplane];
    end

end

% [planequeue, remainderqueue]=pruning(cloud, planequeue, remainderqueue, p); % remove too small planes

plot_test_HM3(cloud, planequeue); % display the planes after segmentation
plot_test_HM3(cloud, remainderqueue); % display the small region after segmentation
%}
%}
%% 
% 180; -180;
% -1.9367; -1.57397; -1.30476; -0.871566; -0.57881; -0.180617; 0.088762; 0.451829; 0.80315; 1.20124; 1.49388; 1.83324; 2.20757; 2.54663; 2.87384; 3.23588; 3.53933; 3.93585; 4.21552; 4.5881; 4.91379; 5.25078; 5.6106; 5.9584; 6.32889; 6.67575; 6.99904; 7.28731; 7.67877; 8.05803; 8.31047; 8.71141; 9.02602; 9.57351; 10.0625; 10.4707; 10.9569; 11.599; 12.115; 12.5621; 13.041; 13.4848; 14.0483; 14.5981; 15.1887; 15.6567; 16.1766; 16.554; 17.1868; 17.7304; 18.3234; 18.7971; 19.3202; 19.7364; 20.2226; 20.7877; 21.3181; 21.9355; 22.4376; 22.8566; 23.3224; 23.971; 24.5066; 24.9992
%% 
base_dir  = 'C:\Users\Administrator\Downloads\scenario1';
nimages = length(dir(fullfile(sprintf('%s/',base_dir), '*.png')));
frame = 0;
img = imread(sprintf('%s/scan%05d.png',base_dir,frame));
img_size = size(img);
g.nscans = img_size(1);
g.nranges = img_size(2);
img_matrix = cast(img, 'double')/500;
g.NUM_EDGES = 9;
g.NO_EDGE = -1;

% img1 = imread(sprintf('%s/scan%05d_SLIC.png',base_dir,frame));
% img = img;
disp('======= Down Sampling     =======');
g.deci_s = 1;
g.deci_r = 1;
tic
[deci_img, g] = down_sample_img(img_matrix, g);
toc

g.nnodes = g.nscans * g.nranges;
nodes = zeros(g.nnodes, 8);
edges = zeros(g.nnodes, 5);

figure;
subplot(2,1,1);
image(deci_img);

disp('======= RangeImage to XYZ =======');
tic
% [nodes] = read_image_to_nodes(img_matrix, base_dir, frame, g);
[nodes] = read_image_to_nodes(deci_img, g);
toc
% nodes = flipud(nodes);


%%
disp('======= Build Graph       =======');
tic
[nodes(:,5:7), edges] = build_graph(nodes(:,1:4), g);
%{
for s = 0:1:nscans-1
    for r=0:1:nranges-1
        idx = s* nranges + r;
        %       nodes(idx+1,1) = img_matrix(s+1, r+1);
        
        if (s == 0 || s == nscans-1 || r == 0 || r == nranges-1 || img_matrix(s+1, r+1) == 0)
            for i=0:1:g.NUM_EDGES-1
                edges(idx+1, i+1) = g.NO_EDGE;
            end
        else
            edges(idx+1, 1) = (s + 0) * nranges + (r + 0)+1;
            edges(idx+1, 2) = (s + 0) * nranges + (r - 1)+1;
            edges(idx+1, 3) = (s - 1) * nranges + (r - 1)+1;
            edges(idx+1, 4) = (s - 1) * nranges + (r + 0)+1;
            edges(idx+1, 5) = (s - 1) * nranges + (r + 1)+1;
            % Remove Edges if range == 0
            if img_matrix(s+1+0, r+1-1) == 0
                edges(idx+1, 2) = g.NO_EDGE;
            end
            if img_matrix(s+1-1, r+1-1) == 0
                edges(idx+1, 3) = g.NO_EDGE;
            end
            if img_matrix(s+1-1, r+1+0) == 0
                edges(idx+1, 4) = g.NO_EDGE;
            end
            if img_matrix(s+1-1, r+1+1) == 0
                edges(idx+1, 5) = g.NO_EDGE;
            end
            % Comput normals
            nodes(idx+1, 4:6) = compute_normal(s,r, nranges, nodes);
           
        end
    end
end
%}
toc
% figure;
subplot(2,1,2);
image(img_matrix);
% drawnow;
frame = frame + 1;


%% test
% figure(3);
nstart = 1;
nend = g.nnodes;
plot_nodes(nodes, nstart, nend)

%%
% clc
% a = [3, 1, 3];
% if a(1,1:3) == 3
%     display('test');
% end

