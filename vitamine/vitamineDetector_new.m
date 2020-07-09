
classdef vitamineDetector_new
   properties
       detectorParameter = struct(...
           'filterType','sobel',...
           'localMaximaWindowSize',30,...
           'localMaximaThreshold',99,...
           'localMaximaGamma',0.1,...
           'GMKernelSigma',2,...
           'hillClimbingLambda',2,...
           'cameraParameter',[]...
       );
       inputImage;
       resizedInputImage;
       previousImage;
       curv;
       subsampledCurv;
       previousLocalMaxima;
       currentLocalMaxima;
       subsampledLocalMaxima;
       previousSubsampledLocalMaxima;
       currentDescriptors;
       currentFeatures;
       previousDescriptors;
       previousFeatures;
       matchesInPreviousFrame;
       matchesInCurrentFrame;
       affineTransformation;
       affineA;
       affineb;
       affinePredictions;
       xt0;
       xt1;
       rotation;
       translation;
       validPoints;
       inliersXt0;
       inliersXt1;
       worldPoints;
       cameraMatrix;
       proposed_location;
       proposed_orientation;
       previousResizedInputImage;
       orientation={};
       location={};
       ViewID;
       minNumPoints;
       numToInit;
       numInTrack=0;
       step=0;
       vSet;
       prevPoints;
       ifInitialized=0;
       worldPointsLastFrame;
       tri_inliersXt0;
       tri_inliersXt1;
   end
   methods
       
       function obj = vitamineDetector_new(inputImage,filterType,camera,minNumPoints)
          obj.detectorParameter.filterType = filterType;
          obj.vSet = imageviewset();
          obj.minNumPoints = minNumPoints;%1000?
          obj.numToInit = obj.minNumPoints;
          obj.orientation{1} = eye(3);
          obj.location{1} = zeros(1,3);
          obj.ViewID=0;
          obj.detectorParameter.cameraParameter = camera;
          obj.cameraMatrix = cameraMatrix(obj.detectorParameter.cameraParameter, eye(3), [0 0 0]);
          if ndims(inputImage)==3
              obj.inputImage = rgb2gray(inputImage);
          else
               obj.inputImage = inputImage;
          end 
          obj.curv = obj.computeCurvature();
%           [obj.resizedInputImage,obj.subsampledCurv] = obj.subSample();
          obj.currentLocalMaxima = obj.computeLocalMaxima(obj.curv,0);
%           obj.subsampledLocalMaxima = obj.computeLocalMaxima(obj.subsampledCurv,1);
          obj.subsampledLocalMaxima = obj.computeLocalMaxima(obj.curv,1);
%           obj.subsampledLocalMaxima = obj.currentLocalMaxima...
%                (randi(length(obj.currentLocalMaxima(:,1)),[ceil(length(obj.currentLocalMaxima(:,1))/6),1]),:);
          [obj.currentFeatures,obj.currentDescriptors] = obj.computeDescriptor();
          obj.previousImage = obj.inputImage;
          obj.prevPoints = obj.currentLocalMaxima;
       end
       
       function r = updateDetector(obj,inputImage,step)
           obj.step = step;
           obj.ViewID = [obj.ViewID step];
           obj.previousDescriptors = obj.currentDescriptors;
           obj.previousFeatures = obj.currentFeatures;
           obj.previousResizedInputImage = obj.resizedInputImage;
           obj.previousLocalMaxima = obj.currentLocalMaxima;
           obj.previousSubsampledLocalMaxima = obj.subsampledLocalMaxima;
           obj.prevPoints = obj.tri_inliersXt1;
           %obj.prevPoints = obj.inliersXt1;
           obj.previousImage = obj.inputImage;
           if(obj.ifInitialized==0)
               obj.numInTrack=0;
           else
               obj.numInTrack = length(obj.prevPoints(:,1));
           end
           obj.numToInit = obj.minNumPoints - obj.numInTrack;
           
           if ndims(inputImage)==3
               obj.inputImage = rgb2gray(inputImage); 
           else
               obj.inputImage = inputImage;
           end
           obj.curv = obj.computeCurvature();
%            [obj.resizedInputImage,obj.subsampledCurv] = obj.subSample();
           obj.currentLocalMaxima = obj.computeLocalMaxima(obj.curv,0);
%            obj.subsampledLocalMaxima = obj.computeLocalMaxima(obj.subsampledCurv,1);
           obj.subsampledLocalMaxima = obj.computeLocalMaxima(obj.curv,1);
%            obj.subsampledLocalMaxima = obj.currentLocalMaxima...
%                (randi(length(obj.currentLocalMaxima(:,1)),[ceil(length(obj.currentLocalMaxima(:,1))/6),1]),:);
           [obj.currentFeatures,obj.currentDescriptors] = obj.computeDescriptor();
           figure(10)
           showMatchedFeatures(obj.inputImage,obj.inputImage,...
               obj.currentLocalMaxima,obj.currentLocalMaxima);
%            imagesc(obj.curv);
           title('Extrema of the curvature');
           figure(1)
           showMatchedFeatures(obj.inputImage,obj.inputImage,...
               obj.currentDescriptors.Location,obj.currentDescriptors.Location);
           title('subsampled local maxima');
           [obj.matchesInPreviousFrame,obj.matchesInCurrentFrame]=obj.matcher();

           figure(2);
            a = obj.previousImage;
            b = obj.inputImage;
            x = max(size(a,1),size(b,1));
            y = max(size(a,2),size(b,2));
            im = zeros(2*x,y);
            im(1:size(a,1),1:size(a,2)) = a;
            im(x+1:x+size(b,1),1:size(b,2)) = b;
            imshow(uint8(im));
            featurePoints_xt0 = obj.matchesInPreviousFrame;
            featurePoints_xt1 = obj.matchesInCurrentFrame;

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
            title('subsampled local maxima matches');
           
           
           
           obj.affineTransformation = obj.fitAffineModel();
           obj.affineA = double(obj.affineTransformation.T(1:2,1:2));
           obj.affineb = double(obj.affineTransformation.T(3,1:2));
           
           numInit = min(obj.numToInit,length(obj.previousLocalMaxima(:,1)));
           idx = randi(length(obj.previousLocalMaxima(:,1)),[numInit,1]);
           obj.prevPoints = [obj.prevPoints;obj.previousLocalMaxima(idx,:)];
           
           obj.affinePredictions = obj.predictPositions();
           [obj.xt1,obj.xt0,obj.validPoints] = obj.hillClimbing();
           
           idx1 = find(obj.validPoints);%idx of validpoints in obj.prevPoints
           idx2 = idx1(idx1<=obj.numInTrack);
           if(isempty(idx2)) %If lose track of all points, reinitialize it again
               obj.ifInitialized = 0;
               if(step~=1)
                   disp('Track lost, reinitializing');
               end
           end
           if(~obj.ifInitialized)
               [obj.rotation,obj.translation,obj.inliersXt0,obj.inliersXt1,...
                   inliersIndex,obj.proposed_orientation,obj.proposed_location]...
                   = obj.poseEstimation();
               obj.orientation{end+1} = obj.proposed_orientation;
               obj.location{end+1} = obj.proposed_location;
               [obj.worldPointsLastFrame,obj.tri_inliersXt0,obj.tri_inliersXt1,idx_tri] = obj.triangulateValidPoints();
               obj.vSet = addView(obj.vSet,step,rigid3d(obj.proposed_orientation,obj.proposed_location),'Points',obj.tri_inliersXt1);
               if(step>=2)
                   idx0 = find(idx_tri); %idx of inliers in obj.inliersXt1
                   idx1 = find(inliersIndex); %idx of obj.inliersXt1 in validpoints
                   idx2 = find(obj.validPoints);%idx of validpoints in obj.prevPoints
                   idx3 = idx2(idx1(idx0)); %idx of inliers in obj.prevPoints
                   idx4 = idx3(idx3<=obj.numInTrack);%?????????????????
                   %idx4 = idx3;
                   matchIndex = [idx4 (1:length(idx4))'];
                   obj.vSet = addConnection(obj.vSet,step-1,step,'Matches',matchIndex);
               end
               obj.ifInitialized = 1;
               figure(6)
               showMatchedFeatures(obj.previousImage,obj.inputImage,obj.tri_inliersXt0,obj.tri_inliersXt1,'blend');
               title('after rejecting far map points');
           else
               %%% solve PnP here instead
               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               validWorldPoints = obj.worldPointsLastFrame(idx2,:);
               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               validImagePoints = obj.xt1(1:length(idx2),:);
               validImagePoints_last = obj.xt0(1:length(idx2),:);
               %%%
               warningstate = warning('off','vision:ransac:maxTrialsReached');
               [obj.proposed_orientation,obj.proposed_location,p3pinliersIndex] = ...
                   estimateWorldCameraPose(validImagePoints,validWorldPoints,...
                   obj.detectorParameter.cameraParameter.Intrinsics,...
                   'Confidence',99,'MaxReprojectionError',1,'MaxNumTrials',2000);
               warning(warningstate);
               p3pInliersXt0 = validImagePoints_last(p3pinliersIndex,:);
               p3pInliersXt1 = validImagePoints(p3pinliersIndex,:);
               p3pWorldPoints = validWorldPoints(p3pinliersIndex,:);
               figure(4)
               showMatchedFeatures(obj.previousImage,obj.inputImage,obj.xt0,obj.xt1,'blend');
               title('affine predictions');
               figure(5)
               showMatchedFeatures(obj.previousImage,obj.inputImage,validImagePoints_last,validImagePoints,'blend');
               title('points used for solving P3P');
               figure(6)
               showMatchedFeatures(obj.previousImage,obj.inputImage,p3pInliersXt0,p3pInliersXt1,'blend');
               title('inliers of solveP3P');
               %%%% triangulate using all points instead ???
               obj.inliersXt0 = obj.xt0(length(idx2)+1:end,:);
               obj.inliersXt1 = obj.xt1(length(idx2)+1:end,:);
               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               %filter out the points in triangulate

               obj.orientation{end+1} = obj.proposed_orientation;
               obj.location{end+1} = obj.proposed_location;
               [newValidWorldPoints,new_tri_inliersXt0,new_tri_inliersXt1,idx_tri] = obj.triangulateValidPoints();
               obj.worldPointsLastFrame = [p3pWorldPoints;newValidWorldPoints];
               obj.tri_inliersXt0 = [p3pInliersXt0;new_tri_inliersXt0];
               obj.tri_inliersXt1 = [p3pInliersXt1;new_tri_inliersXt1];


               figure(7)
               a = obj.previousImage;
                b = obj.inputImage;
                x = max(size(a,1),size(b,1));
                y = max(size(a,2),size(b,2));
                im = zeros(2*x,y);
                im(1:size(a,1),1:size(a,2)) = a;
                im(x+1:x+size(b,1),1:size(b,2)) = b;
                imshow(uint8(im));
                featurePoints_xt0 = obj.tri_inliersXt0;
                featurePoints_xt1 = obj.tri_inliersXt1;
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
%                showMatchedFeatures(obj.previousImage,obj.inputImage,obj.tri_inliersXt0,obj.tri_inliersXt1,'blend');
               title('after rejecting far map points');
               %{
               idx0 = find(idx_tri); %idx of inliers in obj.xt1
               idx1 = find(obj.validPoints); %idx of obj.xt1 in obj.prevPoints
               idx2 = idx1(idx0); %idx of inliers in obj.prevPoints
               idx3 = idx2(idx2<=obj.numInTrack);
               matchIndex = [idx3 (1:length(idx3))'];
%                idx0 = find(idx_tri); %idx of inliers in obj.inliersXt1
%                idx1 = find(inliersIndex); %idx of obj.inliersXt1 in validpoints
%                idx2 = find(obj.validPoints);%idx of validpoints in obj.prevPoints
%                idx3 = idx2(idx1(idx0)); %idx of inliers in obj.prevPoints
% 
%                idx4 = idx3(idx3<=obj.numInTrack);%?????????????????
%                %idx4 = idx3;
%                matchIndex = [idx4 (1:length(idx4))'];
%}
               idxValid = idx1;%find(obj.validPoints);
               idxInTrack = idxValid(1:length(idx2));
               idxMatched = idxInTrack(p3pinliersIndex);
               
               idxNotInTrack = idxValid(length(idx2)+1:end);
               idxNew = idxNotInTrack(idx_tri);
               matchIndex = [idxMatched (1:length(idxMatched))'];
               
               obj.vSet = addView(obj.vSet,step,rigid3d(obj.proposed_orientation,obj.proposed_location),...
                   'Points',obj.tri_inliersXt1);
               obj.vSet = addConnection(obj.vSet,step-1,step,'Matches',matchIndex);
               %%%
           end
           
           %obj.previousImage = obj.inputImage;
           r = obj;
       end
       
       function r = updateCamera(obj,poses,step)
           orientation = poses.AbsolutePose(end).Rotation;
           location = poses.AbsolutePose(end).Translation;
           obj.vSet = updateView(obj.vSet, poses);
           if(step==1)
               obj.cameraMatrix = cameraMatrix(obj.detectorParameter.cameraParameter,...
                   orientation', -location*orientation');
           else
               obj.orientation{end} = orientation;%transposing the cells not the matrix
               obj.location{end} = location;%transposing the cells not the matrix

               obj.cameraMatrix = cameraMatrix(obj.detectorParameter.cameraParameter,...
                   orientation', -location*orientation');
           end
           r=obj;
       end
        
       function r = updateWorldPoints(obj,worldPoints)
           obj.worldPoints = worldPoints;
           r = obj;
       end
       
       function r1 = computeCurvature(obj)
           filter = obj.detectorParameter.filterType;%'sobel'
           image = double(obj.inputImage);
           image = imgaussfilt(image,1);
%            x = fspecial(filter);
%            y = x';
%            fx = imfilter(image,x);
%            fy = imfilter(image,y);
%            fxx = imfilter(fx,x);
%            fyy = imfilter(fy,y);
%            fxy = imfilter(fx,y);
           [fx,fy] = imgradientxy(image,filter);
           [fxx,fxy]= imgradientxy(fx,filter);
           [~,fyy]= imgradientxy(fy,filter);
           r1 = fy.^2.*fxx-2*fx.*fy.*fxy+fx.^2.*fyy;
           r1 = r1./((fx.*fx+fy.*fy).^(3/2)+1e-7);
%            r2 = fy1.^2.*fxx1-2*fx1.*fy1.*fxy1+fx1.^2.*fyy1;
           r1 = imgaussfilt(r1,1);
           %r1 = r2;
           %r1 = mat2gray(r1);
       end
       
       function r = computeLocalMaxima(obj,curv,flag)
           windowsize = obj.detectorParameter.localMaximaWindowSize;
           prominence = max(curv,[],'all')-min(curv,[],'all');
           if(~flag)
               tf = islocalmax(curv,'MinProminence',0.01*prominence,'ProminenceWindow',...
                   windowsize,'MinSeparation',windowsize);
           else
               tf = islocalmax(curv,'MinProminence',0.01*prominence,'ProminenceWindow',...
                   windowsize,'MinSeparation',windowsize*6);
           end
           [row,col] = find(tf);
           r = [col,row]; %(x,y)coordinate
       end
%{
%        function r = computeLocalMaxima(obj,curv,flag) 
%            windowsize = obj.detectorParameter.localMaximaWindowSize;
% %            if(~flag)
% %                windowsize = windowsize*2;
% %            end
%            threshold = obj.detectorParameter.localMaximaThreshold;
%            height = size(curv,1);
%            width = size(curv,2);
%            xBlock = floor(height/windowsize);
%            yBlock = floor(width/windowsize);
%            xEdge = floor((height-windowsize*xBlock)/2);
%            yEdge = floor((width-windowsize*yBlock)/2);
%            r = zeros(1,2);
%            for ii = 1:xBlock
%                for jj = 1:yBlock
%                    localRegion = curv((ii-1)*windowsize+1+xEdge:xEdge+ii*windowsize,...
%                        1+yEdge+(jj-1)*windowsize:yEdge+jj*windowsize);
%                    [~,ind]=max(localRegion(:));
%                    x = floor((ind-1)/windowsize)+1;
%                    y = ind - (x-1)*windowsize;
%                    y = y+(ii-1)*windowsize+xEdge;
%                    x = x+(jj-1)*windowsize+yEdge;
%                    if (curv(y,x)>=(1+obj.detectorParameter.localMaximaGamma)...
%                            *prctile(localRegion,threshold,'all')||flag==1)
%                        if(flag==1)
%                            r = [r;[x(1) y(1)]]; %%%%%%Subsamples are used only for creating ORB points
%                                                 %%%%%%Which requires
%                                                 %%%%%%location in [x,y]
%                        elseif(flag==0)
%                            %r = [r;[y(1) x(1)]];
%                            r = [r;[x(1) y(1)]];
%                        end
%                    end
%                end
%            end
%            r = r(2:end,:);
% %            if(flag==1)
% %                r = FAST_non_max(obj.resizedInputImage,r,0.3);
% %            end
% %            if(flag==0)
% %                r = FAST_non_max(obj.inputImage,r,0.3);
% %            end
%        end
%}
       
       function [resizedInputImage,subsampledCurv] = subSample(obj)
           resizedInputImage = imresize(obj.inputImage,1/3);
           subsampledCurv = imresize(obj.curv,1/3);
       end
       
       function [a,b] = computeDescriptor(obj)
           points = ORBPoints(obj.subsampledLocalMaxima);
           %points = SURFPoints(obj.subsampledLocalMaxima);
           %points = BRISKPoints(obj.subsampledLocalMaxima);
           %points = obj.subsampledLocalMaxima;
           %points = ORBPoints(obj.subsampledLocalMaxima);
           %[a,b] = extractFeatures(obj.resizedInputImage,points);
           %[a,b] = extractFeatures(obj.inputImage,points,'BlockSize',11);
           [a,b] = extractFeatures(obj.inputImage,points);
           %points = ORBPoints(obj.currentLocalMaxima);
           %[a,b] = extractFeatures(obj.inputImage,points);
       end

       function [a,b] = matcher(obj)
           indexPairs = matchFeatures(obj.previousFeatures,obj.currentFeatures,...
               'Method','Approximate','Unique',true,'MaxRatio',1,...
               'MatchThreshold',100);
           a = obj.previousDescriptors(indexPairs(:,1),:);
           b = obj.currentDescriptors(indexPairs(:,2),:);
       end
       
        function r = fitAffineModel(obj)
%              r = estimateGeometricTransform(obj.matchesInPreviousFrame,...
%                  obj.matchesInCurrentFrame,'affine','MaxDistance',2);
                [r,xt0,xt1] = estimateGeometricTransform(obj.matchesInPreviousFrame,...
                 obj.matchesInCurrentFrame,'affine','MaxDistance',25);
                    figure(3);
                    a = obj.previousImage;
                    b = obj.inputImage;
                    x = max(size(a,1),size(b,1));
                    y = max(size(a,2),size(b,2));
                    im = zeros(2*x,y);
                    im(1:size(a,1),1:size(a,2)) = a;
                    im(x+1:x+size(b,1),1:size(b,2)) = b;
                    imshow(uint8(im));

             xt0 = xt0.Location;
             xt1 = xt1.Location;
%              xt0 = xt0;
%              xt1 = xt1;
             hold on;
             for ii = 1:length(xt0(:,1))
                x1 = xt0(ii,1);
                x2 = xt1(ii,1);
                y1 = xt0(ii,2);
                y2 = xt1(ii,2)+x;
                plot([x1,x2],[y1,y2],'Color','green','LineWidth',1);
                plot(x1,y1,'r+','Markersize',5);
                plot(x2,y2,'g+','Markersize',5);
             end
            hold off;
            title('inliers for fitting affine model');
       end
       
       function r = predictPositions(obj)
           r = zeros(size(obj.prevPoints));
           for ii = 1:length(obj.prevPoints(:,1))
               r(ii,:) = obj.prevPoints(ii,:)*(obj.affineA)+obj.affineb;
%                r(ii,:) = transpose(transpose(obj.affineA)*transpose(obj.prevPoints(ii,:))+transpose(obj.affineb));
           end
           
       end
       %{
       function [xt1,xt0,validPoints] = hillClimbing(obj)
           height = size(obj.inputImage,1);
           width = size(obj.inputImage,2);
           xt_bar = ceil(obj.affinePredictions);
           validY=((obj.affinePredictions(:,2)>21)&(obj.affinePredictions(:,2)<=height-21));
           validX=((obj.affinePredictions(:,1)>21)&(obj.affinePredictions(:,1)<=width-21));
           validPoints = validX&validY;
           xt_barx = xt_bar(validPoints,1);
           xt_bary = xt_bar(validPoints,2);
           xt0 = [obj.prevPoints(validPoints,1) obj.prevPoints(validPoints,2)];
           xt1 = [xt_barx xt_bary];       
       end
       %}
      
       function [xt1,xt0,validPoints] = hillClimbing(obj)
           height = size(obj.inputImage,1);
           width = size(obj.inputImage,2);
           xt_bar = ceil(obj.affinePredictions);
           
           max_iter = 20;
           validY=((obj.affinePredictions(:,2)>21)&(obj.affinePredictions(:,2)<=height-21));
           validX=((obj.affinePredictions(:,1)>21)&(obj.affinePredictions(:,1)<=width-21));
           validPoints = validX&validY;
           xt_barx = xt_bar(validPoints,1);
           xt_bary = xt_bar(validPoints,2);
           xt0 = [obj.prevPoints(validPoints,1)...
               obj.prevPoints(validPoints,2)]; 
           xt_bar = [xt_barx xt_bary];
           xt1 = zeros(size(xt_bar));
           neighborIndex = [-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];
%            neighborIndex = [-2 -2;-2 -1;-2 0;-2 1;-2 2;...
%                             -1 -2;-1 -1;-1 0;-1 1;-1 2;...
%                             0 -2;0 -1;0 1;0 2;...
%                             1 -2;1 -1;1 0;1 1;1 2;...
%                             2 -2;2 -1;2 0;2 1;2 2];
           lambda = obj.detectorParameter.hillClimbingLambda;
           GMKernelSigma = obj.detectorParameter.GMKernelSigma;
           for ii = 1:length(xt_bar(:,1))
               coord = xt_bar(ii,:);
               xt1(ii,:) = coord;
               maxF = obj.curv(coord(2),coord(1));
               for jj = 1:max_iter
                   F = zeros(8,1);
                   for kk = 1:8
                       xn = xt1(ii,1)+neighborIndex(kk,1);
                       yn = xt1(ii,2)+neighborIndex(kk,2);
                       if((xn>=1)&&(xn<=width)&&(yn>=1)&&(yn<=height))
                           dist = norm(coord-[xn yn]);
                           F(kk) = obj.curv(yn,xn)+lambda*...
                               (1-dist^2/(dist^2+GMKernelSigma^2));
                       end
                   end
                   [localMax,ind] = max(F);
                   if(localMax>maxF)
                       maxF = localMax;
                       xt1(ii,:) = [xt1(ii,1)+neighborIndex(ind,1)...
                           xt1(ii,2)+neighborIndex(ind,2)];%[x,y]
                   else
                       break;
                   end
               end          
           end
       end
       
       function [rotation, translation,inliersXt0,inliersXt1,inliersIndex,...
               proposed_orientation,proposed_location]= poseEstimation(obj)
           [M,inliersIndex] = estimateEssentialMatrix(obj.xt0,obj.xt1,...
               obj.detectorParameter.cameraParameter,'Confidence',99,'MaxDistance',0.5,'MaxNumTrials',10000);
            inliersXt0 = obj.xt0(inliersIndex,:);
            inliersXt1 = obj.xt1(inliersIndex,:);

%             % Compute homography and evaluate reconstruction
%             intrinsics = obj.detectorParameter.cameraParameter;
%             [tformH, scoreH, inliersIdxH] = obj.computeHomography(obj.xt0, obj.xt1);
% 
%             % Compute fundamental matrix and evaluate reconstruction
%             [tformF, scoreF, inliersIdxF] = obj.computeFundamentalMatrix(obj.xt0, obj.xt1);
%             
%             % Select the model based on a heuristic
%             ratio = scoreH/(scoreH + scoreF);
%             ratioThreshold = 0.45;
%             if ratio > ratioThreshold
%                 inliersIndex = inliersIdxH;
%                 tform          = tformH;
%             else
%                 inliersIndex = inliersIdxF;
%                 tform          = tformF;
%             end
% 
%             % Computes the camera location up to scale. Use half of the 
%             % points to reduce computation
%             inliersXt0  = obj.xt0(inliersIndex,:);
%             inliersXt1 = obj.xt1(inliersIndex,:);
%             [rotation, translation, validFraction] = relativeCameraPose(tform, intrinsics, ...
%                 inliersXt0, inliersXt1);
% 
% 
%             
            figure(4)
            showMatchedFeatures(obj.previousImage,obj.inputImage,obj.xt0,obj.xt1,'blend');
            title('affine predictions');
            figure(5)
            showMatchedFeatures(obj.previousImage,obj.inputImage,inliersXt0,inliersXt1,'blend');
            title('inliers of pose estimation using essential matrix');

            %%%%%%%%% Sometimes the Essential Matrix estimated above is
            %%%%%%%%% unable to select physically realizable rotation
            %%%%%%%%% matrices in the relativeCameraPose function
            [rotation, translation,validFraction] = relativeCameraPose(M,obj.detectorParameter.cameraParameter,inliersXt0,inliersXt1);
            if(ndims(rotation)~=2)
               rotation = rotation(:,:,1);
               translation = translation(1,:);
               warning('More than one relative camera poses');
            end
            proposed_orientation = obj.orientation{end}*rotation;
            proposed_location = obj.location{end}+translation;
       end
       
       function [H, score, inliersIndex] = computeHomography(obj,matchedPoints1, matchedPoints2)

            [H, inlierPoints1, inlierPoints2] = estimateGeometricTransform( ...
                matchedPoints1, matchedPoints2, 'projective', ...
                'MaxNumTrials', 1e3, 'MaxDistance', 4, 'Confidence', 90);

            [~, inliersIndex] = intersect(matchedPoints1, ...
                inlierPoints1, 'row', 'stable');

            locations1 = inlierPoints1;
            locations2 = inlierPoints2;
            xy1In2     = transformPointsForward(H, locations1);
            xy2In1     = transformPointsInverse(H, locations2);
            error1in2  = sum((locations2 - xy1In2).^2, 2);
            error2in1  = sum((locations1 - xy2In1).^2, 2);

            outlierThreshold = 6;

            score = sum(max(outlierThreshold-error1in2, 0)) + ...
                sum(max(outlierThreshold-error2in1, 0));
       end
       
       function [F, score, inliersIndex] = computeFundamentalMatrix(obj,matchedPoints1, matchedPoints2)

            [F, inliersLogicalIndex]   = estimateFundamentalMatrix( ...
                matchedPoints1, matchedPoints2, 'Method','RANSAC',...
                'NumTrials', 1e3, 'DistanceThreshold', 0.01);

            inlierPoints1 = matchedPoints1(inliersLogicalIndex,:);
            inlierPoints2 = matchedPoints2(inliersLogicalIndex,:);

            inliersIndex  = find(inliersLogicalIndex);

            locations1    = inlierPoints1;
            locations2    = inlierPoints2;

            % Distance from points to epipolar line
            lineIn1   = epipolarLine(F', locations2);
            error2in1 = (sum([locations1, ones(size(locations1, 1),1)].* lineIn1, 2)).^2 ...
                ./ sum(lineIn1(:,1:2).^2, 2);
            lineIn2   = epipolarLine(F, locations1);
            error1in2 = (sum([locations2, ones(size(locations2, 1),1)].* lineIn2, 2)).^2 ...
                ./ sum(lineIn2(:,1:2).^2, 2);

            outlierThreshold = 4;

            score = sum(max(outlierThreshold-error1in2, 0)) + ...
                sum(max(outlierThreshold-error2in1, 0));

       end 
       
       function [validWorldPoints,inliersXt0,inliersXt1,inliers_idx] = triangulateValidPoints(obj)
               projMatrix1 = obj.cameraMatrix;
               projMatrix2 = cameraMatrix(obj.detectorParameter.cameraParameter,...
                   obj.proposed_orientation',-obj.proposed_location*obj.proposed_orientation');
               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               [worldPoints,reprojectionErrors] = triangulate(obj.inliersXt0,obj.inliersXt1,projMatrix1,projMatrix2);
               inliers_idx1 = false(size(worldPoints,1),1);
               for ii=1:length(inliers_idx1)
                   dist=norm(worldPoints(ii,:)-obj.proposed_location,2);
                   if(dist>30)
                       inliers_idx1(ii)=0;
                   else
                       inliers_idx1(ii)=1;
                   end
               end
               inliers_idx2 = reprojectionErrors<0.5;
               inliers_idx = inliers_idx1&inliers_idx2;
               validWorldPoints = worldPoints(inliers_idx,:);
               inliersXt0 = obj.inliersXt0(inliers_idx,:);
               inliersXt1 = obj.inliersXt1(inliers_idx,:);
       end
   end
end