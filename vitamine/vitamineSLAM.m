clear variables; close all; clc;

gt = load('D:\workspace\data\dataset\poses\00.txt');

sequencePath = 'D:\workspace\data\dataset\sequences\00\image_0\';
% Projection matrix for kitti/00 camera0
projMatrix = [...
7.188560000000e+02 0.000000000000e+00 6.071928000000e+02;% 0.000000000000e+00;
0.000000000000e+00 7.188560000000e+02 1.852157000000e+02;% 0.000000000000e+00;
0.000000000000e+00 0.000000000000e+00 1.000000000000e+00]';% 0.000000000000e+00];
camera = cameraParameters('IntrinsicMatrix',projMatrix,'ImageSize',[376,1241]);

initIm = 0;
lastIm = 4540;

im0 = takeImage( sequencePath, initIm );
numPoints = 5000;
obj = vitamineDetector_new(im0,'sobel',camera,numPoints);
last_orientation = eye(3);
last_location = zeros(1,3);
movements = [];
trajectory = reshape([last_orientation last_location'],1,12);
current_traj = [last_orientation last_location'];
groundtruth_cache = reshape(gt(1,:),4,3)';
for step=1:lastIm-initIm
    im1 = takeImage( sequencePath, step+initIm );
    obj = obj.updateDetector(im1,step);
    BA;  
%     last_orientation = obj.proposed_orientation
%     last_location = obj.proposed_location
%     current_traj = reshape(current_traj',1,12);
%     trajectory = [trajectory;current_traj];
%     current_move = reshape([obj.rotation obj.translation'],1,12);
%     movements = [movements;current_move];
    current_move = [last_orientation/current_traj(1:3,1:3) last_location'-current_traj(1:3,4)]
    current_traj = [last_orientation last_location']
     
    groundtruth = reshape(gt(step+1,:),4,3)'
    groundtruth_move = [groundtruth(1:3,1:3)/groundtruth_cache(1:3,1:3) groundtruth(1:3,4)-groundtruth_cache(1:3,4)]
    groundtruth_cache = groundtruth;
%     if(step>=2)
         plots;
%     end
    im0 = im1;

    step = step
end

traj = [];
ori = obj.orientation;
loc = obj.location;
for ii = 1:length(ori)
o = cell2mat(ori(ii));
l = cell2mat(loc(ii));
traj = [traj;reshape([o l']',1,12)];
end
figure(3);
hold on;
plot(gt(:,4),gt(:,12),'b.');
plot(traj(:,4),traj(:,12),'r.');

 