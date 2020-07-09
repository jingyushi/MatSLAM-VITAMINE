clear variables; close all; clc;
sequencePath = 'D:\workspace\data\dataset\sequences\00\image_0\';
initIm = 0;
im0 = takeImage( sequencePath, initIm );
projMatrix = [...
7.188560000000e+02 0.000000000000e+00 6.071928000000e+02;% 0.000000000000e+00;
0.000000000000e+00 7.188560000000e+02 1.852157000000e+02;% 0.000000000000e+00;
0.000000000000e+00 0.000000000000e+00 1.000000000000e+00]';% 0.000000000000e+00];
camera = cameraParameters('IntrinsicMatrix',projMatrix,'ImageSize',[376,1241]);
numPoints = 2000;
im0_new = im0;
obj = vitamineDetector_new(im0_new,'sobel',camera,numPoints);
%gt_affine = randomAffine2d('Scale',[1.1,1.1],'XTranslation',[-30,30],'YTranslation',[-30,30]);
%im1 = imwarp(im0,gt_affine);
%im1_new = im1(1:376,1:1241);
im1 = takeImage(sequencePath, initIm+1);
obj = obj.updateDetector(im1,1);
%a = gt_affine.T
b = obj.affineTransformation.T

figure(5);
a = obj.previousResizedInputImage;
b = obj.resizedInputImage;
x = max(size(a,1),size(b,1));
y = max(size(a,2),size(b,2));
im = zeros(2*x,y);
im(1:size(a,1),1:size(a,2)) = a;
im(x+1:x+size(b,1),1:size(b,2)) = b;
imshow(uint8(im));
featurePoints_xt0 = obj.matchesInPreviousFrame.Location;
featurePoints_xt1 = obj.matchesInCurrentFrame.Location;
hold on;
for ii = 1:length(featurePoints_xt0(:,1))
    x1 = featurePoints_xt0(ii,1);
    x2 = featurePoints_xt1(ii,1);
    y1 = featurePoints_xt0(ii,2);
    y2 = featurePoints_xt1(ii,2)+x;
    plot([x1,x2],[y1,y2],'Color','green','LineWidth',1);
    plot(x1,y1,'r+','Markersize',5);
    plot(x2,y2,'g+','Markersize',5);
end
hold off;