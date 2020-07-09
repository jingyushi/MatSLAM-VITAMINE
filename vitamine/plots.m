%{
im1_size = size(im1);
temp = zeros(2*im1_size(1),im1_size(2));
temp(1:im1_size(1),1:im1_size(2))=im1;
temp(im1_size(1)+1:2*im1_size(1),1:im1_size(2))=im0;
temp = uint8(temp);
figure(1);
imshow(temp);
hold on;
for ii=1:length(obj.inliersXt0(:,1))
    x1 = obj.inliersXt1(ii,1);
    x2 = obj.inliersXt0(ii,1);
    y1 = obj.inliersXt1(ii,2)+im1_size(1);
    y2 = obj.inliersXt0(ii,2);
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
    plot(x2,y2,'r+','Markersize',5);
end
%waitforbuttonpress;
hold off;
figure(3);
im = cat(1,obj.resizedInputImage,obj.previousResizedInputImage);
imshow(im);
featurePoints_xt0 = obj.matchesInPreviousFrame.Location;
featurePoints_xt1 = obj.matchesInCurrentFrame.Location;
hold on;
for ii = 1:length(featurePoints_xt0(:,1))
    x1 = featurePoints_xt1(ii,1);
    x2 = featurePoints_xt0(ii,1);
    y1 = featurePoints_xt1(ii,2);
    y2 = featurePoints_xt0(ii,2)+length(obj.resizedInputImage(:,1));
    plot([x1,x2],[y1,y2],'Color','green','LineWidth',1);
end
hold off;
%}

% figure(8);
% 
% xt0 = obj.xt0;
% ap = obj.affinePredictions(obj.validPoints,:);
% hc = obj.xt1;
% %c = mat2gray(obj.curv);
% c = cat(1,obj.previousImage,obj.inputImage);
% %c = (obj.curv);
% offset = size(obj.inputImage,1);
% imshow(c)
% hold on;
% for ii=1:length(hc(:,1))
%     if(mod(ii,5)==1)
%     x1 = xt0(ii,1);
%     x2 = ap(ii,1);
%     x3 = hc(ii,1);
%     y1 = xt0(ii,2);
%     y2 = ap(ii,2)+offset;
%     y3 = hc(ii,2)+offset;
%     plot(x1,y1,'ro','Markersize',5);
%     plot(x2,y2,'bx','Markersize',5);
%     plot(x3,y3,'g+','Markersize',5);
%     plot([x1,x2],[y1,y2],'Color','green','LineWidth',1);
%     plot([x2,x3],[y2,y3],'Color','yellow','LineWidth',1);
%     plot([x1,x3],[y1,y3],'Color','blue','LineWidth',1);
%     end
% end
% 
% hold off;
% if (step>=1)
%     figure(1);
%     showMatchedFeatures(obj.previousImage,obj.inputImage,obj.tri_inliersXt0,obj.tri_inliersXt1,'blend');
%     figure(2);
%     ptCloud = pointCloud(obj.worldPointsLastFrame);
%     
%     grid on;
%     pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%     'MarkerSize', 45);
%     hold on;
%     plotCamera('Location', obj.proposed_location, 'Orientation', obj.proposed_orientation, 'Size', 0.1, ...
%     'Color', 'b', 'Label', num2str(step), 'Opacity', 0);
%     hold off;
%     figure(3)
%     clf;
%     hold on;
%     for ii=1:length(obj.worldPointsLastFrame)
%         plot(obj.worldPointsLastFrame(ii,1),obj.worldPointsLastFrame(ii,3),'r.','Markersize',3);
%     end
%     hold on;
%     plot(obj.proposed_location(1),obj.proposed_location(3),'b+','Markersize',10);
%     hold off;
% end