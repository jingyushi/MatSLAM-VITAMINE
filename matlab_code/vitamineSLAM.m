clear variables; close all; clc;

sequencePath = 'D:\workspace\data\dataset\sequences\00\image_0\';
% Projection matrix for kitti/00 camera0
projMatrix = [...
7.188560000000e+02 0.000000000000e+00 6.071928000000e+02;% 0.000000000000e+00;
0.000000000000e+00 7.188560000000e+02 1.852157000000e+02;% 0.000000000000e+00;
0.000000000000e+00 0.000000000000e+00 1.000000000000e+00];% 0.000000000000e+00];
camera = cameraParameters('IntrinsicMatrix',projMatrix);

initIm = 0;
lastIm = 1000;
im0 = takeImage( sequencePath, initIm );
im_size = size(im0);
height = im_size(1);
width=im_size(2);
region = im0(21:height-20,21:width-20);
obj = vitamineDetector(region,'sobel',camera);
accu_rotation = eye(3);
accu_translation = zeros(3,1);

for step=initIm+1:lastIm
    im1 = takeImage( sequencePath, step );
    obj = obj.updateDetector(im1);
    %[~,refinedPose] = bundleAdjustment(obj.worldPoints,
    %disp([obj.rotation obj.translation']);
    accu_rotation = accu_rotation*obj.rotation
    accu_translation = accu_translation + obj.translation'
    im1_size = size(im1);
    temp = zeros(2*im1_size(1),im1_size(2));
    temp(1:im1_size(1),1:im1_size(2))=im1;
    temp(im1_size(1)+1:2*im1_size(1),1:im1_size(2))=im0;
    temp = uint8(temp);
    figure(1);
    imshow(temp);
    hold on;
    for ii=1:length(obj.inliersXt0(:,1))
        x1 = obj.inliersXt0(ii,1);
        x2 = obj.inliersXt1(ii,1);
        y1 = obj.inliersXt0(ii,2)+im1_size(1);
        y2 = obj.inliersXt1(ii,2);
        plot([x1,x2],[y1,y2],'Color','green','LineWidth',1);
        %plot(x2,y2,'r+','Markersize',10);
    end
    %waitforbuttonpress;
    hold off;
    figure(2);
    imshow(im1);
    hold on;
    for ii=1:length(obj.inliersXt0(:,1))
        x1 = obj.inliersXt0(ii,1);
        x2 = obj.inliersXt1(ii,1);
        y1 = obj.inliersXt0(ii,2);
        y2 = obj.inliersXt1(ii,2);
        plot([x1,x2],[y1,y2],'Color','green','LineWidth',1);
        plot(x2,y2,'r+','Markersize',10);
    end
    %waitforbuttonpress;
    hold off;
    im0 = im1;
    step = step
end

 