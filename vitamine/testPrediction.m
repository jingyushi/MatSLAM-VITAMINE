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
curv0 = obj.curv;
im1_new = takeImage( sequencePath, initIm+1 );
obj = obj.updateDetector(im1_new,1);

prevPoints = obj.prevPoints;
affinePredictions = obj.affinePredictions;
validPoints = obj.validPoints;
finalCorrection = obj.xt1;
figure(1)
showMatchedFeatures(im0_new,im1_new,prevPoints(validPoints,:),finalCorrection,'blend');
hold on;
for ii=1:length(obj.currentLocalMaxima(:,1))
    plot(obj.currentLocalMaxima(ii,1),obj.currentLocalMaxima(ii,2),'b+','Markersize',3);
end
hold off;
figure(2)
showMatchedFeatures(im0_new,im1_new,prevPoints(validPoints,:),affinePredictions(validPoints,:),'blend');
hold on;
for ii=1:length(obj.currentLocalMaxima(:,1))
    plot(obj.currentLocalMaxima(ii,1),obj.currentLocalMaxima(ii,2),'b+','Markersize',3);
end
hold off;



% curv1 = obj.curv;
% figure(1)
% curv = cat(1,curv0,curv1);
% imshow(mat2gray(curv));
% hold on;
% jj=0;
% for ii=1:length(obj.validPoints)
%     if(obj.validPoints(ii)==1)
%         jj=jj+1;
%     end
%     if(mod(ii,20)==1)
%     plot(obj.affinePredictions(ii,1),obj.affinePredictions(ii,2)+size(curv1,1),'r+','Markersize',3);
%     plot(obj.prevPoints(ii,1),obj.prevPoints(ii,2),'g+','Markersize',3);
%     if(obj.validPoints(ii)==1)
%         x0 = obj.prevPoints(ii,1);
%         x1 = obj.affinePredictions(ii,1);
%         x2 = obj.xt1(jj,1);
%         y0 = obj.prevPoints(ii,2);
%         y1 = obj.affinePredictions(ii,2)+size(curv1,1);
%         y2 = obj.xt1(jj,2)+size(curv1,1);
%         plot(x2,y2,'b+','Markersize',3);
%         plot([x0,x2],[y0,y2],'Color','red','LineWidth',1);
%         %plot([x0,x1],[y0,y1],'Color','green','LineWidth',1);
%         plot([x1,x2],[y1,y2],'Color','blue','LineWidth',1);
%     end
%     end
% end
% hold off;
% figure(2)
% im = cat(1,im0_new,im1_new);
% imshow(im);
% hold on;
% jj=0;
% for ii=1:length(obj.validPoints)
%     if(obj.validPoints(ii)==1)
%         jj=jj+1;
%     end
%     if(mod(ii,20)==1)
%     plot(obj.affinePredictions(ii,1),obj.affinePredictions(ii,2)+size(curv1,1),'r+','Markersize',3);
%     plot(obj.prevPoints(ii,1),obj.prevPoints(ii,2),'g+','Markersize',3);
%     if(obj.validPoints(ii)==1)
%         x0 = obj.prevPoints(ii,1);
%         x1 = obj.affinePredictions(ii,1);
%         x2 = obj.xt1(jj,1);
%         y0 = obj.prevPoints(ii,2);
%         y1 = obj.affinePredictions(ii,2)+size(curv1,1);
%         y2 = obj.xt1(jj,2)+size(curv1,1);
%         plot(x2,y2,'b+','Markersize',3);
%         plot([x0,x2],[y0,y2],'Color','red','LineWidth',1);
%         %plot([x0,x1],[y0,y1],'Color','green','LineWidth',1);
%         plot([x1,x2],[y1,y2],'Color','blue','LineWidth',1);
%     end
%     end
% end
% hold off;