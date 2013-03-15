% KITTI RAW DATA DEVELOPMENT KIT
% 
% Demonstrates projection of the velodyne points into the image plane

% clear and close everything
clear all; close all; clc;
disp('======= KITTI DevKit Demo =======');
addpath C:\Users\Administrator\Documents\GitHub\matpcl
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
% base_dir  = 'C:\Users\Administrator\Downloads\scenario2';
nimages = length(dir(fullfile(sprintf('%s/',base_dir), '*.png')));
% frame = 11041;
frame = 90;
img = imread(sprintf('%s/scan%05d.png',base_dir,frame));

% img_dir = 'C:\Users\Administrator\Downloads\scenario1_part2\image_01\data';
% original_img = imread(sprintf('%s/00000%05d.png',img_dir,frame));

% image(img_matrix);
[g.nscans, g.nranges] = size(img);
g.yaw_resolution = 360/g.nranges;
img_matrix = cast(img, 'double')/500;
% img_matrix = img_matrix/max(img_matrix(:));
g.NUM_EDGES = 4;
g.NO_EDGE = nan;
clear img
%% Down Sampling
% img1 = imread(sprintf('%s/scan%05d_SLIC.png',base_dir,frame));
% img = img;
disp('======= Down Sampling =======');
g.deci_s = 1;
g.deci_r = 1;
% g.img_scan_start = 1;
% g.img_scan_end = 870;
g.img_scan_start = 300;
g.img_scan_end = 400;
tic
[deci_img, g] = down_sample_img(img_matrix, g);
toc
%% RangeImage to Node
g.nnodes = g.nscans * g.nranges;
nodes = zeros(g.nnodes, 10);
edges = zeros((g.nscans-1)*(g.nranges-1)*g.NUM_EDGES, 5);
sorted_edges = zeros((g.nscans-1)*(g.nranges-1)*g.NUM_EDGES, 6);

disp('======= RangeImage to XYZ =======');
tic
% [nodes] = read_image_to_nodes(img_matrix, base_dir, frame, g);
[nodes(:,1:4)] = read_image_to_nodes(deci_img, g);
toc
% nodes = flipud(nodes);
%% Gradient calculation
disp('======= Compute Gradient Features =======');
tic
% [A, B] = gradient(deci_img);
% % g = (abs(A)*2+abs(B)*1.5);
% g_mag = sqrt(A.^2+B.^2);
% c = atan2(B,A);

%     myfilter = fspecial('gaussian',[3 3], 0.8);
%     deci_img = filter2(myfilter, deci_img);
g.label.ground = 255;
g.label.objects = 1;
[g_mag, g_dir, gx, gy] = imgradient(deci_img);
[g_mag1] = imgradient(g_dir.*180/3.14);
g_mag_original = g_mag1;

for s = 1:1:g.nscans
    for r=1:1:g.nranges
        if g_mag1(s, r) <= 20 && deci_img(s,r) ~= 0
            g_mag1(s, r) = -180;
        else
            g_mag1(s, r) = 0;
        end
    end
end
g.num_ground = 0;
toc
g_mag3 = zeros(g.nscans, g.nranges);
for s = 0:1:g.nscans-1
   for r=0:1:g.nranges-1
      idx = s* g.nranges + r;
      nodes(idx+1, 8) = g_dir(s+1,r+1);
      % Label Assign - ground and objects
      % Direction of Gradient of image for flat surface
      if (g_dir(s+1, r+1).*180/3.14 <= -80) && (g_dir(s+1, r+1).*180/3.14 >= -100)
          g_mag3(s+1, r+1) = 1; 
      else
          nodes(idx+1, 10) = g.label.objects;
          g_mag3(s+1, r+1) = 0; 
      end
        % Magnitude of directional gradient image for flat surface
%         if g_mag1(s+1,r+1) == -180
%             nodes(idx+1, 10) = g.label.gound;
%             g.num_ground = g.num_ground + 1;
%         else
%             nodes(idx+1, 10) = g.label.objects;
%         end
   end
end
clear idx;clear s;clear r;
%% Build Graph
disp('======= Build Graph =======');
% node
% 1: x, 2: y, 3: z, 4: r, 
% 5:nx, 6:ny, 7:nz, 8:dir_grad ,
% 9:ccs 10: objest Label(0: ground 1~: objects)
tic
[nodes(:,5:7), edges] = build_graph(nodes(:,1:4), nodes(:,10), g);
% nodes(:,5) = smooth(nodes(:,5));
% nodes(:,6) = smooth(nodes(:,6));
% nodes(:,7) = smooth(nodes(:,7));
toc
%% Clear empty edge and resizing edge
edges(~any(edges,2),:) = [];
g.nedges = size(edges,1);
%% Save temp_edges
temp_edges = zeros(g.nedges, 4);
temp_edges = edges(:,1:3);
%% Compute Edge Weights
% idx = ~(edges(:,1) == 0 & edges(:,2) == 0 & edges(:,3) == 0);
disp('======= Compute Edge Weights =======');
% edge info.
% 1: a, 2: b, 3: w_dis, 4: w_normal or w_gradient 5:z-axis diff
g.max_d_weight = 0.15;
% g.max_d_weight = 0.05;
g.w = 0.7;
g.height_weight = 0.05;
tic
for i=1:1:g.nedges
    if nodes(edges(i,1),4) == 0 || nodes(edges(i,2),4) == 0 || nodes(edges(i,1),10) == 0 || nodes(edges(i,2),10) == 0 
        edges(i,3) = g.NO_EDGE;
        edges(i,4) = g.NO_EDGE;
        edges(i,5) = g.NO_EDGE;
    else
%         edges(i,3) = sqrt((nodes(edges(i,1),1) - nodes(edges(i,2),1))^2 + (nodes(edges(i,1),2) - nodes(edges(i,2),2))^2 + (nodes(edges(i,1),3) - nodes(edges(i,2),3))^2 );
        edges(i,3) = abs(nodes(edges(i,1),4) - nodes(edges(i,2),4));
        % normals
%         edges(i,4) = 1 - abs(nodes(edges(i,1),5:7) * transpose(nodes(edges(i,2),5:7)));
        % gradient
        edges(i,4) = abs(nodes(edges(i,1),8) - nodes(edges(i,2),8));
        diff_height = abs(nodes(edges(i,1),3) - nodes(edges(i,2),3));
        if edges(i,3) > min((g.max_d_weight*nodes(edges(i,1),4)), (g.max_d_weight*nodes(edges(i,2),4))) ...
            || diff_height > min((g.height_weight*nodes(edges(i,1),4)), (g.height_weight*nodes(edges(i,2),4)))
        %                 || diff_height > 0.2
    %         if edges(i,3) > (g.max_d_weight)
                edges(i,3) = g.NO_EDGE;
                edges(i,4) = g.NO_EDGE;
                %         edges(i,5) = g.NO_EDGE;
            
        end
        if edges(i,3) ~= g.NO_EDGE && isnan(edges(i,3)) ~= 1
%             edges(i,5) = (1-edges(i,4))*edges(i,4) + (1-edges(i,4))*(edges(i,3)/g.max_d_weight);
            edges(i, 5) = abs(nodes(edges(i,1),3) - nodes(edges(i,2),3));
        else
            edges(i,5) = g.NO_EDGE;
        end
%         if edges(i,5) > g.height_weight
%             edges(i,3) = g.NO_EDGE;
%             edges(i,4) = g.NO_EDGE;
%             edges(i,5) = g.NO_EDGE;
%         end
    end
    
%     if edges(i,3) > (round(edges(i,1)/870)+1 * (1/64) )
end
toc

frame = frame + 1;

%% Segment Graph
disp('======= Segment Graph =======');
g.min_size = 10;
% g.num_ccs = 0;
g.c = 20.0;
% normals
% g.n = 0.5;
% gradient
g.n = 1.5;
g.nnc = 0.5;
% g.method = 4;
% [nodes(:,9), g] = segment_graph(nodes(:,4), edges, g);

g.method = 3;
[nodes(:,9), g, sorted_edges, threshold] = segment_graph(nodes(:,4), edges, g, nodes(:,10));
disp(g.num_ccs);
clear sorted_edges;
clear threshold;
%% Original Gray Image plot
% figure;
% imshow(original_img);
%% gradient image plot
%{
figure;
subplot(2,1,1);
image(abs(gx)*1000);
subplot(2,1,2);
image(abs(gy)*1000);

%%
% for s = 1:1:g.nscans
%    for r=1:1:g.nranges
%        g_mag1(s,r) = g_mag(s,r)*s/1.5;
%    end
% end

%}
%% gradient and flat surface image plot
g_mag2 = zeros(g.nscans, g.nranges);
for s = 1:1:g.nscans
    for r=1:1:g.nranges
        if g_mag1(s, r) == -180
            g_mag2(s, r) = 1;
        else
            g_mag2(s, r) = 0;
        end
    end
end
tic
threshold = graythresh(g_mag2);
originalImage = im2bw(g_mag2, threshold);
%  originalImage = medfilt2(originalImage,[37 37],'symmetric'); 
 originalImage = bwareaopen(originalImage,10);
 toc
%  L = bwlabel(originalImage,8);
 [L, NUM] = bwlabeln(originalImage,8);
 NUM
 
 %  originalImage = medfilt2(originalImage,[37 37],'symmetric'); 
 originalImage1 = bwareaopen(g_mag3,20);
 toc
%  L = bwlabel(originalImage,8);
 [L1, NUM1] = bwlabeln(originalImage1,8);
 NUM1
figure;
subplot(5,1,1);
imagesc(g_mag1);
% g_mag1
title('Original flat surface');
subplot(5,1,2);
imagesc(originalImage);
title('noise rejected flat surface');
subplot(5,1,3);
imagesc(L);
title(' Labeled noise rejected flat surface');
subplot(5,1,4);
imagesc(g_dir.*180/3.14);
title('directional gradient image');
subplot(5,1,5);
imagesc(L1);
title(' Ground surface only');
clear g_mag2;
%% Plot various image
% %{
disp('======= Plot image =======');
tic
plot_image(deci_img, nodes, g_dir, g, g_mag.*180/3.14);
toc
%  %}
%% z axis image plot
%{
for s = 0:1:g.nscans-1
    for r=0:1:g.nranges-1
        idx = s* g.nranges + r;
        zaxis_img(s+1,r+1) = nodes(idx+1,3)*-1;
    end
end
figure;
imagesc(zaxis_img);
%}
%%
% 
% figure;
% imagesc(abs(g_dir.*180/pi));
% % %
% g.method = 5;
%%
% % tic
% [nodes(:,10), g] = segment_graph(nodes(:,4), edges, g);
% toc
% disp('======= plot image  =======');
% tic
% plot_image(deci_img, nodes(:,10), g);
% toc 
%%
%{
disp('======= plot segment clouds  =======');
tic
figure;
hold on;
axis equal;
c1 = colormap(jet(100));
r1 = randperm(100);
% h = plot3(nodes(1,1),nodes(1,2),nodes(1,3)*-1, '.', 'color', c1(r1(nodes(1,8)),1:3));
xlim([-80, 80]);
ylim([-80, 80]);
zlim([-3, 3]);
% set(gca,'LegendColorbarListeners',[]); 
% setappdata(gca,'LegendColorbarManualSpace',1);
% setappdata(gca,'LegendColorbarReclaimSpace',1);
set(gcf,'Visible','off');
% set(h,'EraseMode','none');
% plot3(nodes(1,1),nodes(1,2),nodes(1,3)*-1, '.', 'color', c1(r1(nodes(1,8)),1))
% colorbar;
for i = 1:1:g.nnodes
    if(nodes(i,4) ~= 0)
        plot3(nodes(i,1),nodes(i,2),nodes(i,3)*-1, '.', 'color', c1(r1(mod(nodes(i,8),100)+1),1:3))
%         set(h, 'xdata', nodes(i,1), 'ydata', nodes(i,2), 'zdata', nodes(i,3), 'color', c1(r1(nodes(i,8)),1:3))
    end
end
set(gcf,'Visible','on');
% delete c;
% delete r;
toc
%}
      
%% Plot Node
% figure(3);
% g.nstart = 1;
% g.nend = g.nnodes;
% plot_nodes(nodes, g.nstart, g.nend)

%% Plot Graph
%{
disp('======= plot graph  =======');
tic
plot_graph(nodes(:,1:3), edges(:,1:3), g);
toc
%}
%% plot label2rgb
%{
% imshow(label2rgb(nodes(:,9)));
% clc
% a = [3, 1, 3];
% if a(1,1:3) == 3
%     display('test');
% end
%}
%% gradient img plot
%{
% figure;
% H = fspecial('disk',0.5);
% sharpened = imfilter(deci_img,H,'replicate');
% subplot(2,2,4); 
% image(sharpened); title('Sharpened Image');
% tic
% [m_i, d_i] = imgradient(g_dir);
% toc
% figure;
% subplot(2,1,1);
% imagesc(d_i);
% subplot(2,1,2);
% imagesc(m_i);

% figure;
% [m_i, d_i] = imgradient(g_dir);
% subplot(2,1,1);
% imagesc(d_i);
% subplot(2,1,2);
% imagesc(m_i);
%}
%% edge plot
%{
% bw = edge(deci_img.*500);
% figure;
% image(bw);
% imshowpair(m_i, d_i, 'montage');
% figure;
% d = smooth(c);
% myfilter = fspecial('gaussian',[3 3], 0.5);
% d = filter2(myfilter, c);
% imshow(g_mag);
% colormap(lines(100));
% quiver(g, 'color', 'r');
%}
%% gradient vector flow plot
%{
[rows, columns] = size(deci_img);
[dx, dy] = gradient(double(deci_img));
[x y] = meshgrid(1:columns, 1:rows);
% c = atan2(B,A);
u = dx;
v = dy;
image(deci_img);
hold on
quiver(u, v, 'color', 'w')
%}
%%
% gg = bwmorph(deci_img,'thin',Inf);
% figure,imagesc(gg);
%{
gg = edge(deci_img, 'canny');
figure;
imagesc(gg);
%}
%% remove unused value
clear threshold;
clear sorted_edges;
% clear gx;
% clear gy;
%{
clc;
d = zeros(g.nedges,2);
disp('======= c1  =======');
tic
for i=1:1:g.nedges
    if nodes(edges(i,1),4) == 0 || nodes(edges(i,2),4) == 0
        edges(i,3) = g.NO_EDGE;
    else
%         edges(i,3) = pdist([nodes(edges(i,1),1:3); nodes(edges(i,2),1:3)], 'euclidean');
        edges(i,3) = sqrt((nodes(edges(i,1),1) - nodes(edges(i,2),1))^2 + (nodes(edges(i,1),2) - nodes(edges(i,2),2))^2 + (nodes(edges(i,1),3) - nodes(edges(i,2),3))^2 );
    end
    edges(i,4) = 1 - abs(nodes(edges(i,1),5:7) * transpose(nodes(edges(i,2),5:7)));
end
toc
disp('======= c2  =======');
tic
for i=1:1:g.nedges
    if nodes(edges(i,1),4) == 0 || nodes(edges(i,2),4) == 0
        d(i,1) = g.NO_EDGE;
        d(i,2) = g.NO_EDGE;
    else
%         d(i,1) = sqrt((nodes(edges(i,1),1) - nodes(edges(i,2),1))^2 + (nodes(edges(i,1),2) - nodes(edges(i,2),2))^2 + (nodes(edges(i,1),3) - nodes(edges(i,2),3))^2 );
        d(i,1) = hello(nodes(edges(i,1),1:3), nodes(edges(i,2),1:3));
        d(i,2) = 1 - abs(nodes(edges(i,1),5:7) * transpose(nodes(edges(i,2),5:7)));
    end;
    
end
toc
%}
%% plot3k
%{
disp('======= plot color point clouds  =======');
k = mod(nodes(:,9), g.num_ccs);
figure('color','white');
plot3k({nodes(:,1)*-1 nodes(:,2) nodes(:,3)*-1},...
    'ColorData',k,'ColorRange',[0 g.num_ccs],'Marker',{'o',3},...
    'Labels',{'Peaks','Radius','','Intensity','Lux'},...
    'PlotProps',{'FontSize',12});
axis equal;
clear k;
%}
%% plot grid mesh using plot_test_HM2
% cloud = nodes(:,1:3);
%{
plane.index=1:size(nodes(:,1:3),1);
plane.level=1;
p.sensortype = 1; % 1: LRF
disp('segmenting...');

tic
[plane, p] = firstsegmentation(nodes(:,1:3),plane,p);
toc
%}
% plot_test_HM2(nodes(:,1:3), plane); % display the result after the first segmentation

%% plot point clouds in pcd_viewer
% %{
display('======= PCD Viewer =======');
k = mod(nodes(:,9), g.num_ccs)+1;
fpcd = figure;
rgb_colormap = colormap(jet(g.num_ccs));
rgb_color = rgb_colormap(k,:);
close(fpcd);
points = [transpose(nodes(:,1:3)); transpose(rgb_color); transpose(nodes(:, 5:7))];
pclviewer(points, '-ps 2 -ax 1');
clear k; clear rgb_colormap;clear rgb_color;
clear points;
% %}
%% Edge detection
% %{
temp_nodes = zeros(g.nnodes, 1);
% temp_edges = zeros(g.nedges, 1);


% Edge label Description %
% 0: range value == 0, 1:edge 존재, 2: 0 pixel의 우 혹은 하 pixel labeling
% g.NO_EDGE: no edges
g.edge_label.ZERO_RANGE = 0;
g.edge_label.OBJECT_EDGE = 1;
g.edge_label.FAR_EDGE = 2;
g.edge_label.NEIBOR_ZERO_RANGE = 3;
 % Assign start and end of row pixel to to edge;
g.edge_label.START_N_END_ROW = 4;
% Assign flat surface directional gradient image
g.edge_label.FLAT_SURFACE = 5;
g.edge_label.NON_FLAT_BOUNDARY = 6;
g.min_d_weight = 0.08;
g.raster_weight = 0.005;
g.radial_weight = 0.25;
g.height_weight = 0.02;
% Build Graph with Range Weight
for i=1:1:g.nedges

    if nodes(temp_edges(i,1),4) == 0 && nodes(temp_edges(i,2),4) ~= 0
        temp_nodes(temp_edges(i,2),1) = g.edge_label.NEIBOR_ZERO_RANGE;
    elseif nodes(temp_edges(i,1),4) == 0 % Zero Range node
        temp_edges(i,4) = g.NO_EDGE;
    elseif nodes(temp_edges(i,2),4) == 0 %neighbor edge zero range node
        temp_edges(i,4) = 2;
    else
        % Compute height(z-value) difference
        diff_height = abs(nodes(temp_edges(i,1),3) - nodes(temp_edges(i,2),3));
        % Compute range difference
        temp_edges(i,3) = abs(nodes(temp_edges(i,1),4) - nodes(temp_edges(i,2),4));
        min_range = min((nodes(temp_edges(i,1),4)), (nodes(temp_edges(i,2),4)));
%         if min_range > 10.0
        if (temp_edges(i,2) - temp_edges(i,1)) == 1
            weighted_range = g.raster_weight * min_range;
        elseif (temp_edges(i,1) + g.nranges == temp_edges(i,2))
            weighted_range = (g.raster_weight/g.radial_weight) * min_range;
        else
            weighted_range = g.radial_weight * min_range;
        end
        weighted_height = g.height_weight * min_range;
%             weighted_height = 0;
%         else
%             min_range = g.min_d_weight * min_range;
%         end
        if temp_edges(i,3) > weighted_range ...
%                 || diff_height > weighted_height
            temp_edges(i,4) = g.NO_EDGE;
%             if temp_edges(i,2) == 0
%                 temp_edges(i,4) = 2;
%             end
        else
            temp_edges(i,4) = 1;
        end
    end
end

for i=1:1:g.nedges
        if isnan(temp_edges(i,4)) == 1
            % Edge assign 1: object edge, 2: unocupanncy edge
            if nodes(temp_edges(i,1),4) <= nodes(temp_edges(i,2),4)
                short_node_num = temp_edges(i,1);
                long_node_num = temp_edges(i,2);
            else
                short_node_num = temp_edges(i,2);
                long_node_num = temp_edges(i,1);
            end
            temp_nodes(short_node_num) = g.edge_label.OBJECT_EDGE;
            temp_nodes(long_node_num) = g.edge_label.FAR_EDGE;
        elseif temp_edges(i,4) == 0
%             temp_nodes(temp_edges(i,1),1) = 1.5;
            temp_nodes(temp_edges(i,1),1) = g.edge_label.ZERO_RANGE;
        elseif temp_edges(i,4) == 2
            temp_nodes(temp_edges(i,1),1) = g.edge_label.NEIBOR_ZERO_RANGE;
        end
        %             temp_img(s+1,r+1) = nodes(idx+1,1);
end
% %% Edge Image
temp_img = zeros(g.nscans, g.nranges);
g.label.ground  = 0;
for s = 0:1:g.nscans-1
    for r=0:1:g.nranges-1
        idx = s* g.nranges + r;
        % Assign start and end of row pixel to to edge;
        if (s == 0) || (s == g.nscans-1) % || (r == g.nranges-1)
           temp_nodes(idx+1, 1) = g.edge_label.START_N_END_ROW; 
        end
        temp_img(s+1,r+1) = temp_nodes(idx+1,1);
%         if nodes(idx+1, 10) == g.label.ground && temp_img(s+1, r+1) == 0
        % Assign flat surface magnitude of directional gradient image
%         if g_mag1(s+1, r+1) == -180 && temp_img(s+1, r+1) == 0
        % Assign flat surface directional gradient image
        if originalImage1(s+1, r+1) == 1 && temp_img(s+1, r+1) == g.edge_label.ZERO_RANGE
            temp_img(s+1,r+1) = g.edge_label.FLAT_SURFACE;
        end
        %             temp_img(s+1,r+1) = nodes(idx+1,1);
    end
end
%{
% for s = 1:1:g.nscans
%     for r=1:1:g.nranges
%         if g_mag1(s, r) <= 20
%             g_mag1(s, r) = -180;
%         else
%             g_mag1(s, r) = 0;
%         end
%     end
% end

% figure;
% subplot(2,1,1);
% image(deci_img);
% title('Original Img, deci\_img');
% subplot(2,1,2);
% % colormap(lines);
% imagesc(temp_img);
title('Edge Img and Ground Plane, temp\_img');
%}
temp_img1 = zeros(g.nscans, g.nranges);
%     cm = colormap(jet(100));
ncolor = randperm(100);
for s = 0:1:g.nscans-1
    for r=0:1:g.nranges-1
        idx = s* g.nranges + r;
        if nodes(idx+1,9) ~= 0
            temp_img1(s+1,r+1) = ncolor(mod(nodes(idx+1,9), 100)+1);
        end
        %             temp_img(s+1,r+1) = nodes(idx+1,1);
    end
end

for s = 1:1:g.nscans
    for r=1:1:g.nranges
        if temp_img(s,r) ~= g.edge_label.ZERO_RANGE
            temp_img1(s,r) = 0;
        end
        %             temp_img(s+1,r+1) = nodes(idx+1,1);
    end
end
%     %{
    f = figure;
    fullscreen = get(0,'ScreenSize');
    % fullscreen = [x_start y_start x_end y_end]
    set(f, 'Position',[0, 0, fullscreen(3)/2, fullscreen(4)]);
    subplot(3,1,1);
%     colormap(jet(100));
    imagesc(temp_img1);
    subplot(3,1,2);
    imagesc(temp_img);
    subplot(3,1,3);
    imagesc(deci_img);
%     %}
    
%% New Segment Graph
%{
g.c = 10.0;
disp('===== Segment Graph =====');
tic
[nodes(:,9), g] = segment_graph(nodes(:,4), temp_edges, g, nodes(:,10));
toc
%%
disp('======= Plot image =======');
tic
plot_image(deci_img, nodes, g_dir, g, g_mag.*180/3.14);
toc
%% plot point clouds in pcd_viewer
% %{
display('======= PCD Viewer =======');
k = mod(nodes(:,9), g.num_ccs)+1;
fpcd = figure;
rgb_colormap = colormap(jet(g.num_ccs));
rgb_color = rgb_colormap(k,:);
close(fpcd);
points = [transpose(nodes(:,1:3)); transpose(rgb_color); transpose(nodes(:, 5:7))];
pclviewer(points, '-ps 2 -ax 1');
clear k; clear rgb_colormap;clear rgb_color;
clear points;
% %}
%}
     %% Boundary extraction of ground image using matlab edge func
     %{
     filtered_edge_img = bwareaopen(temp_img,10);
     edge_n_flat = (filtered_edge_img.*100) + L;
     BW = edge(originalImage,'prewitt');
     figure;
     subplot(3,1,1);
     imagesc(temp_img);
     subplot(3,1,2);
     imagesc(filtered_edge_img);
     subplot(3,1,3);
     imagesc(deci_img);
     %}

 
     %{
     figure;
     subplot(4,1,1);
     imagesc(originalImage1);
     title('Original flat surface');
     subplot(4,1,2);
     
     imagesc(BW1);
     title('boundary of original flat surface');
     subplot(4,1,3);
     
     imagesc(BW2);
     title('Original flat surface and it''s boundary');
     subplot(4,1,4);
     
     imagesc(deci_img);
     title('Original range image');
     for s = 1:1:g.nscans
         for r=1:1:g.nranges
             if BW1(s,r) ~= 0
                 temp_img(s,r) = 5;
             end
             %             temp_img(s+1,r+1) = nodes(idx+1,1);
         end
     end
     %}
% %}

%% Boundary Extraction of non flat area
     BW1 = bwperim(originalImage1, 8);
     BW2 = BW1*2 + temp_img;
     tic
     BW3 = xor(originalImage1, 1);
     toc
%      tic
%      BW5 = ~originalImage1;
%      toc
     filtered_BW3 = bwareaopen(BW3,10,8);
     BW4 = bwperim(filtered_BW3, 4);
     temp_img2 = zeros(g.nscans, g.nranges);
     tic
     % non-flat ground boundary
     for s = 1:1:g.nscans
         for r=1:1:g.nranges
             if filtered_BW3(s,r) == 0
                 temp_img2(s,r) = 1;
             end
             if BW4(s,r) == 1
                 temp_img2(s,r) = 5;
             end
             
             %             temp_img(s+1,r+1) = nodes(idx+1,1);
         end
     end
     toc
     % flat surface and it's boundary plot
     %{
     figure;
     subplot(4,1,1);
     imagesc(BW3);
     title('none flat surface, BW3');
     subplot(4,1,2);
     imagesc(filtered_BW3);
     title('noise rejected non flat surface, filtered_BW3');
     subplot(4,1,3);
     imagesc(BW4);
     title('Boundary of noise rejected non flat surface, BW4');   
     subplot(4,1,4);
     imagesc(temp_img2);
     title('Flat surface + Boundary of non flat surfance');   
     %}

     % Compare range based edge detection and boundary of flat surface detection
     figure;
     subplot(3,1,1);
     imagesc(temp_img);
     title('Edge labeling image, temp\_img');
     subplot(3,1,2);
     imagesc(temp_img2);
     title('Flat surface + Boundary of non flat surfance, temp\_img2');    
     subplot(3,1,3);
     imagesc(deci_img);
     title('Original Image, deci\_img');


%% Plot Graph
% %{
disp('======= plot graph  =======');
tic
plot_graph(nodes(:,1:3), [temp_edges(:,1:2) temp_edges(:,4)], g);
toc
% %}

%% V disparity map
%{
max_r = round(max(deci_img(:))+0.5);
v_img = zeros(g.nscans, max_r);
for s = 1:1:g.nscans
    for r=1:1:g.nranges
        range = round(deci_img(s,r)+0.5);
        if deci_img(s,r) ~= 0
           v_img(s, range) = v_img(s, range) + 1;
        end
    end
end

figure;
% subplot(1,2,1);
imagesc(deci_img);
% subplot(1,2,2);
figure;
imagesc(v_img);
%}

%% Line Detetion
% w = [ -1 -1 -1; 2 2 2; -1 -1 -1];
% g = imfilter(deci_img, w);
% figure;
% image(g);

%% Edge gradient compare sobel, prewitt, roberts and original 
%{
[max_range  ]= max(max(deci_img));
bw_image = deci_img./max_range;
[im1, im1_thresh, im1_gv, im1_gh]= edge(bw_image, 'sobel');
% im1 = edge(deci_img, 'sobel');
im1_mag = sqrt(im1_gv.^2+im1_gh.^2);
% g_dir = atan2(B,A).*180/3.14;
im1_dir = atan2(im1_gh,im1_gv).*180/3.14;
figure;
subplot(5,1,1);
colormap(JET(100));
imagesc(g_mag);
title('own');
subplot(5,1,2);
imagesc(im1_mag);
title('sobel');
subplot(5,1,3);
imagesc(g_dir.*-180/3.14);
title('own');
subplot(5,1,4);
imagesc(im1_dir);
title('sobel');
%}

%% gradient edge plot( Original img, DoG img, MDoG img);
%{
figure;
subplot(3,1,1);
imagesc(deci_img);
title('Original Image, deci\_img');
subplot(3,1,2);
imagesc(g_dir);
title('Directional Gradient Image, g\_dir');
subplot(3,1,3);
imagesc(g_mag_original);
title('Magnitude of Directional Gradient Image, g\_mag\_original');
%}

%% Gx, Gy, MGoD, DGoD Computation
%{
GoD_x = zeros(g.nscans, g.nranges);
GoD_y = zeros(g.nscans, g.nranges);
%%% Computation of gx
% Computes gx of first row and end row
tic
[gx gy] = gradient(deci_img);
toc
pause(0.5);
tic
for s = 1:1:g.nscans
    GoD_x(s,1) = deci_img(s,2) - deci_img(s,1);
    GoD_x(s,g.nranges) = deci_img(s,g.nranges) - deci_img(s,g.nranges-1);
end
% Computes gx of 2nd row to (end-1) row
for s = 1:1:g.nscans
    for r=2:1:g.nranges-1
        GoD_x(s,r) = (deci_img(s,r+1) - deci_img(s,r-1));
    end
end
% Computes gy of first colum and end colum
for r = 1:1:g.nranges
    GoD_y(1,r) = deci_img(2,r) - deci_img(1,r);
    GoD_y(g.nscans,r) = deci_img(g.nscans, r) - deci_img(g.nscans-1,r);
end
% Computes gy of 2nd colum to (end-1) colum
for s = 2:1:g.nscans-1
    for r=1:1:g.nranges
        GoD_y(s,r) = (deci_img(s+1,r) - deci_img(s-1,r));
    end
end
toc
% plot gx, gy, DoG_x, DoG_y
%{
figure;
subplot(4,1,1);
imagesc(gx);
title('matlab ver. gx');
subplot(4,1,2);
imagesc(GoD_x);
title('own ver. gx');
subplot(4,1,3);
imagesc(gy);
title('matlab ver. gy');
subplot(4,1,4);
imagesc(GoD_y);
title('own ver. gy');
%}


MGoD = sqrt(GoD_x.^2+GoD_y.^2);
DGoD = atan2(GoD_y,GoD_x).*180/3.14;
figure;
subplot(4,1,1);
imagesc(g_mag);
title('matlab ver. g\_mag');
subplot(4,1,2);
imagesc(MGoD);
title('own ver. MGoD');
subplot(4,1,3);
imagesc(g_dir.*180/3.14);
title('matlab ver. g\_dir');
subplot(4,1,4);
imagesc(DGoD);
title('own ver. DGoD');

%% DGoD Computation
for s = 1:1:g.nscans
    for r=2:1:g.nranges-1
        GoD_x(s,r) = (deci_img(s,r+1) - deci_img(s,r-1));
    end
end
%}

%% DGoD Computation
%{
DGoD = zeros(g.nscans, g.nranges);
for s = 1:1:g.nscans
    for r=1:1:g.nranges
        if gx(s,r) ~= 0 && gy(s,r) == 0
%            if gx(s,r) > 0
%                DGoD(s,r) = pi;
%            else
%                DGoD(s,r) = -pi;
%            end
        elseif gy(s,r) == 0 && gx(s,r) == 0
            DGoD(s,r) = 0;
        else
            DGoD(s,r) = atan2(gy(s,r), gx(s,r))*-1;
        end
        
    end
end

figure;
subplot(4,1,1);
imagesc(gx);
title('matlab ver. g\_mag');
subplot(4,1,2);
imagesc(gy);
title('own ver. MGoD');
subplot(4,1,3);
imagesc(g_dir.*180/3.14);
title('matlab ver. g\_dir');
subplot(4,1,4);
imagesc(DGoD.*180/3.14);
title('own ver. DGoD');
%}
%% Plot point clouds in pcd_viewer
% %{
display('======= PCD Viewer =======');
for s = 0:1:g.nscans-1
   for r=0:1:g.nranges-1
      idx = s* g.nranges + r;
      % flat & objects & boundary of nonflat area
      nodes(idx+1, 10) = temp_img(s+1,r+1);
      % magnitude of directional gradient
%       nodes(idx+1, 10) = floor(g_mag_original(s+1,r+1));
   end
end
k = nodes(:,10)+1;
fpcd = figure;
rgb_colormap = colormap(jet(max(k(:))));
rgb_color = rgb_colormap(k,:);
close(fpcd);
points = [transpose(nodes(:,1:3)); transpose(rgb_color); transpose(nodes(:, 5:7))];
pclviewer(points, '-ps 2 -ax 1');
clear k; clear rgb_colormap;clear rgb_color;
clear points;
% %}


%% Plot continous Images
%{
base_dir  = 'C:\Users\Administrator\Downloads\scenario2';
nimages = length(dir(fullfile(sprintf('%s/',base_dir), '*.png')));
% frame = 11041;
frame = 0;
f = figure;
set(f, 'keypressfcn', @stop_img);
for i=55:1:200
   img = imread(sprintf('%s/scan%05d.png',base_dir,i));
   fprintf('%i ',i);

   imagesc(img);
   pause(0.5);
end
%}

%% Normailized Gx, Gy Computation
%{
GoD_x = zeros(g.nscans, g.nranges);
GoD_y = zeros(g.nscans, g.nranges);
%%% Computation of gx
% Computes gx of first row and end row
tic
for s = 1:1:g.nscans
    GoD_x(s,1) = (deci_img(s,2) - deci_img(s,1))/max(deci_img(s,2),deci_img(s,1));
    GoD_x(s,g.nranges) = (deci_img(s,g.nranges) - deci_img(s,g.nranges-1))/max(deci_img(s,g.nranges),deci_img(s,g.nranges-1));
end
% Computes gx of 2nd row to (end-1) row
for s = 1:1:g.nscans
    for r=2:1:g.nranges-1
        GoD_x(s,r) = (deci_img(s,r+1) - deci_img(s,r-1))/max(deci_img(s,r+1),deci_img(s,r-1));
    end
end
% Computes gy of first colum and end colum
for r = 1:1:g.nranges
    GoD_y(1,r) = (deci_img(2,r) - deci_img(1,r))/max(deci_img(2,r), deci_img(1,r));
    GoD_y(g.nscans,r) = (deci_img(g.nscans, r) - deci_img(g.nscans-1,r))/max(deci_img(g.nscans, r), deci_img(g.nscans-1,r));
end
% Computes gy of 2nd colum to (end-1) colum
for s = 2:1:g.nscans-1
    for r=1:1:g.nranges
        GoD_y(s,r) = (deci_img(s+1,r) - deci_img(s-1,r))/max(deci_img(s+1,r), deci_img(s-1,r));
    end
end
toc
% plot gx, gy, DoG_x, DoG_y
% %{
figure;
subplot(4,1,1);
imshow(gx);
title('matlab ver. gx');
subplot(4,1,2);
imshow(GoD_x);
title('own ver. gx');
subplot(4,1,3);
imagesc(gy);
title('matlab ver. gy');
subplot(4,1,4);
imagesc(GoD_y);
title('own ver. gy');
MGoD = sqrt(GoD_x.^2+GoD_y.^2);
DGoD = atan2(GoD_y,GoD_x).*180/3.14;
figure;
subplot(4,1,1);
imagesc(g_mag);
title('matlab ver. g\_mag');
subplot(4,1,2);
imagesc(MGoD);
title('own ver. MGoD');
subplot(4,1,3);
imagesc(g_dir.*180/3.14);
title('matlab ver. g\_dir');
subplot(4,1,4);
imagesc(DGoD);
title('own ver. DGoD');
%}

%% Normailized Gx, Gy Ratio Computation
%{
GoD_x = zeros(g.nscans, g.nranges);
GoD_y = zeros(g.nscans, g.nranges);
%%% Computation of gx
% Computes gx of first row and end row
tic
for s = 1:1:g.nscans 
    GoD_x(s,1) = abs(deci_img(s,2) - deci_img(s,1));%/max(deci_img(s,2),deci_img(s,1));
    GoD_x(s,g.nranges) = abs(deci_img(s,g.nranges) - deci_img(s,g.nranges-1));%/max(deci_img(s,g.nranges),deci_img(s,g.nranges-1));
    if GoD_x(s, 1) > 1
        GoD_x(s,1) = 1;
    end
    if GoD_x(s,g.nranges) > 1
        GoD_x(s,g.nranges) = 1;
    end
end
% Computes gx of 2nd row to (end-1) row
for s = 1:1:g.nscans
    for r=2:1:g.nranges-1
        if deci_img(s,r+1) ~= 0 && deci_img(s,r) ~= 0 && deci_img(s,r-1)~=0
            max_value = max(deci_img(s,r+1),deci_img(s,r-1));
            nx2 = deci_img(s,r+1)/max_value;
            nx1 = deci_img(s,r)/max_value;
            nx0 = deci_img(s,r-1)/max_value;
            gx1 = nx2 - nx1+0.000005;
            gx2 = nx0 - nx1+0.000005;
            GoD_x(s,r) = gx1/gx2;
            if GoD_x(s, r) > 1
                GoD_x(s,r) = 1;
            end
            if GoD_x(s, r) < 1
                GoD_x(s,r) = -1;
            end
%             if GoD_x(s, r) < 0.005
%                 GoD_x(s,r) = 0;
%             end
        end
    end
end
% Computes gy of first colum and end colum
% for r = 1:1:g.nranges
%     GoD_y(1,r) = (deci_img(2,r) - deci_img(1,r))/max(deci_img(2,r), deci_img(1,r));
%     GoD_y(g.nscans,r) = (deci_img(g.nscans, r) - deci_img(g.nscans-1,r))/max(deci_img(g.nscans, r), deci_img(g.nscans-1,r));
% end
% % Computes gy of 2nd colum to (end-1) colum
% for s = 2:1:g.nscans-1
%     for r=1:1:g.nranges
%         GoD_y(s,r) = (deci_img(s+1,r) - deci_img(s-1,r))/max(deci_img(s+1,r), deci_img(s-1,r));
%     end
% end
toc
% plot gx, gy, DoG_x, DoG_y
% %{
figure;
subplot(4,1,1);
imagesc(gx);
title('matlab ver. gx');
subplot(4,1,2);
imagesc(GoD_x);
title('own ver. gx');
subplot(4,1,3);
imagesc(deci_img);
title('matlab ver. gy');
subplot(4,1,4);
imagesc(g_dir);
title('own ver. gy');
%}

%% Height Gradient Image Computation
DGoH = zeros(g.nscans, g.nranges);
MGoH = zeros(g.nscans, g.nranges);
height_img = zeros(g.nscans, g.nranges);
display('======= PCD Viewer =======');
for s = 0:1:g.nscans-1
   for r=0:1:g.nranges-1
      idx = s* g.nranges + r;
      % flat & objects & boundary of nonflat area
      height_img(s+1, r+1) = nodes(idx+1, 3);
      % magnitude of directional gradient
%       nodes(idx+1, 10) = floor(g_mag_original(s+1,r+1));
   end
end
[MGoH, DGoH hgx hgy] = imgradient(height_img);
figure;
subplot(4,1,1);
imagesc(hgx);
title('gx of height img');
subplot(4,1,2);
imagesc(hgy);
title('gy of height img');
subplot(4,1,3);
imagesc(g_dir);
title('DGoD of Range img');
subplot(4,1,4);
imagesc(DGoH.*180/3.14);
title('DGoH');