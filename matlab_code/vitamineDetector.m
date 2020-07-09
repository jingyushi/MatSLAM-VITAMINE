
classdef vitamineDetector
   properties
       detectorParameter = struct(...
           'filterType','sobel',...
           'localMaximaWindowSize',8,...
           'localMaximaThreshold',95,...
           'localMaximaGamma',0.05,...
           'GMKernelSigma',1,...
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
       pointsToPredict;
       xt1_new;
       xt0_new;
       validPoints_new;
       inliersXt0;
       inliersXt1;
       worldPoints;
       cameraMatrix;
       reproErrors;
   end
   methods
       
       function obj = vitamineDetector(inputImage,filterType,camera)
          obj.detectorParameter.filterType = filterType;
          obj.detectorParameter.cameraParameter = camera;
          obj.cameraMatrix = [obj.detectorParameter.cameraParameter.IntrinsicMatrix [0;0;0]];
          if ndims(inputImage)==3
              obj.inputImage = rgb2gray(inputImage);
          else
               obj.inputImage = inputImage;
          end
          
          obj.curv = obj.computeCurvature();
          [obj.resizedInputImage,obj.subsampledCurv] = obj.subSample();
%           imgsize = size(obj.subsampledCurv);
%           obj.detectorParameter.localMaximaWindowSize = min(floor(imgsize(1)/12),floor(imgsize(2)/12));
          obj.currentLocalMaxima = obj.computeLocalMaxima(obj.curv,0);
          obj.subsampledLocalMaxima = obj.computeLocalMaxima(obj.subsampledCurv,1);
          [obj.currentFeatures,obj.currentDescriptors] = obj.computeDescriptor();
          obj.previousImage = obj.inputImage;
          obj.xt0 = obj.currentLocalMaxima;
          %obj.pointsToPredict = 
       end
       
       function r = updateDetector(obj,inputImage)
           obj.previousDescriptors = obj.currentDescriptors;
           obj.previousFeatures = obj.currentFeatures;
           %obj.previousImage = obj.inputImage;
           obj.previousLocalMaxima = obj.currentLocalMaxima;
           obj.previousSubsampledLocalMaxima = obj.subsampledLocalMaxima;
           if ndims(inputImage)==3
               obj.inputImage = rgb2gray(inputImage); 
           else
               obj.inputImage = inputImage;
           end
           obj.curv = obj.computeCurvature();
           [obj.resizedInputImage,obj.subsampledCurv] = obj.subSample();
           obj.currentLocalMaxima = obj.computeLocalMaxima(obj.curv,0);
           obj.subsampledLocalMaxima = obj.computeLocalMaxima(obj.subsampledCurv,1);
           [obj.currentFeatures,obj.currentDescriptors] = obj.computeDescriptor();
           [obj.matchesInPreviousFrame,obj.matchesInCurrentFrame]=obj.matcher();
           obj.affineTransformation = obj.fitAffineModel();
           obj.affineA = double(obj.affineTransformation.T(1:2,1:2));
           obj.affineb = double(obj.affineTransformation.T(3,1:2))*6;
%            A = obj.affineTransformation.T
           obj.affinePredictions = obj.predictPositions();
           [obj.xt1,obj.xt0,obj.validPoints] = obj.hillClimbing();
           %[obj.xt1,obj.xt0,obj.validPoints] = obj.gaussNewton(2);
           [obj.rotation,obj.translation,obj.inliersXt0,obj.inliersXt1] = obj.poseEstimation();
           %[obj.worldPoints,obj.reproErrors] = obj.triangulateValidPoints();
           obj.previousImage = obj.inputImage;
           r = obj;
       end
       
       function r = pointsPrediction(obj,points,inputImage)
           obj.previousDescriptors = obj.currentDescriptors;
           obj.previousFeatures = obj.currentFeatures;
           %obj.previousImage = obj.inputImage;
           obj.previousLocalMaxima = obj.currentLocalMaxima;
           obj.previousSubsampledLocalMaxima = obj.subsampledLocalMaxima;
           if ndims(inputImage)==3
               obj.inputImage = rgb2gray(inputImage); 
           else
               obj.inputImage = inputImage;
           end
           [obj.curv,obj.curv_smooth] = obj.computeCurvature();
           [obj.resizedInputImage,obj.subsampledCurv] = obj.subSample();
           obj.currentLocalMaxima = obj.computeLocalMaxima(obj.curv_smooth,0);
           obj.subsampledLocalMaxima = obj.computeLocalMaxima(obj.subsampledCurv,1);
           [obj.currentFeatures,obj.currentDescriptors] = obj.computeDescriptor();
           [obj.matchesInPreviousFrame,obj.matchesInCurrentFrame]=obj.matcher();
           obj.affineTransformation = obj.fitAffineModel();
           obj.affineA = double(obj.affineTransformation.T(1:2,1:2));
           obj.affineb = double(obj.affineTransformation.T(3,1:2))*2;
           obj.pointsToPredict = points;
           obj.affinePredictions = obj.predictPositions();%[x,y]
           [obj.xt1,obj.xt0,obj.validPoints] = obj.hillClimbing();
           [obj.xt1_new,obj.xt0_new,obj.validPoints_new] = obj.gaussNewton(2);
           %[obj.rotation,obj.translation] = obj.poseEstimation();
           obj.previousImage = obj.inputImage;
           r = obj;
       end
       
       function r1 = computeCurvature(obj)
           filter = obj.detectorParameter.filterType;
           image = obj.inputImage;
           image = imgaussfilt(image,1);
           [fx,fy] = imgradientxy(image,filter);
           [fxx,fxy]= imgradientxy(fx,filter);
           [~,fyy]= imgradientxy(fy,filter);
           r1 = fy.^2.*fxx-2*fx.*fy.*fxy+fx.^2.*fyy;
           r1 = imgaussfilt(r1,1);
           %r1 = r2;
           %r = mat2gray(r);
       end
       
       function r = computeLocalMaxima(obj,curv,flag) 
           windowsize = obj.detectorParameter.localMaximaWindowSize;
%            if(~flag)
%                windowsize = windowsize*2;
%            end
           threshold = obj.detectorParameter.localMaximaThreshold;
           height = size(curv,1);
           width = size(curv,2);
           xBlock = floor(height/windowsize);
           yBlock = floor(width/windowsize);
           xEdge = floor((height-windowsize*xBlock)/2);
           yEdge = floor((width-windowsize*yBlock)/2);
           r = zeros(1,2);
           for ii = 1:xBlock
               for jj = 1:yBlock
                   localRegion = curv((ii-1)*windowsize+1+xEdge:xEdge+ii*windowsize,...
                       1+yEdge+(jj-1)*windowsize:yEdge+jj*windowsize);
                   [~,ind]=max(localRegion(:));
                   x = floor((ind-1)/windowsize)+1;
                   y = ind - (x-1)*windowsize;
                   y = y+(ii-1)*windowsize+xEdge;
                   x = x+(jj-1)*windowsize+yEdge;
                   if (curv(y,x)>=(1+obj.detectorParameter.localMaximaGamma)...
                           *prctile(localRegion,threshold,'all'))%||flag==1)
                       if(flag==1)
                           r = [r;[x(1) y(1)]]; %%%%%%Subsamples are used only for creating ORB points
                                                %%%%%%Which requires
                                                %%%%%%location in [x,y]
                       elseif(flag==0)
                           %r = [r;[y(1) x(1)]];
                           r = [r;[x(1) y(1)]];
                       end
                   end
               end
           end
           r = r(2:end,:);
       end
       
       function [resizedInputImage,subsampledCurv] = subSample(obj)
           resizedInputImage = imresize(obj.inputImage,1/6);
           subsampledCurv = imresize(obj.curv,1/6);
       end
       
       function [a,b] = computeDescriptor(obj)
           points = ORBPoints(obj.subsampledLocalMaxima);
           [a,b] = extractFeatures(obj.resizedInputImage,points);
           %points = ORBPoints(obj.currentLocalMaxima);
           %[a,b] = extractFeatures(obj.inputImage,points);
       end

       function [a,b] = matcher(obj)
           indexPairs = matchFeatures(obj.previousFeatures,obj.currentFeatures,...
               'Method','Approximate','Unique',true);
           a = obj.previousDescriptors(indexPairs(:,1));
           b = obj.currentDescriptors(indexPairs(:,2));
       end
       
       function r = fitAffineModel(obj)
             r = estimateGeometricTransform(obj.matchesInPreviousFrame,obj.matchesInCurrentFrame,'affine');
       end
       
       function r = predictPositions(obj)
%            r = zeros(size(obj.pointsToPredict));
%            temp = obj.pointsToPredict;
%            %temp(:,[1,2]) = temp(:,[2,1]);
%            for ii = 1:length(temp)
%                r(ii,:) = transpose(obj.affineA*transpose(temp(ii,:))+transpose(obj.affineb));%[x,y]
%            end
           r = zeros(size(obj.previousLocalMaxima));
           for ii = 1:length(obj.previousLocalMaxima(:,1))
               r(ii,:) = transpose(obj.affineA*transpose(obj.previousLocalMaxima(ii,:))+transpose(obj.affineb));
           end
           
       end
       
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
%            xt0 = [obj.pointsToPredict(validPoints,1)...
%                obj.pointsToPredict(validPoints,2)]; 
           xt0 = [obj.previousLocalMaxima(validPoints,1)...
               obj.previousLocalMaxima(validPoints,2)]; 
           xt_bar = [xt_barx xt_bary];
           xt1 = zeros(size(xt_bar));
           neighborIndex = [-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];
           for ii = 1:length(xt_bar(:,1))
               coord = xt_bar(ii,:);
               xt1(ii,:) = coord;
               maxF = obj.curv(coord(2),coord(1));
               lambda = 0.001;
               for jj = 1:max_iter
                   F = zeros(8,1);
                   for kk = 1:8
                       xn = xt1(ii,1)+neighborIndex(kk,1);
                       yn = xt1(ii,2)+neighborIndex(kk,2);
                       if((xn>=1)&&(xn<=width)&&(yn>=1)&&(yn<=height))
                           dist = norm(coord-[xn yn]);
                           F(kk) = obj.curv(yn,xn)+lambda*...
                               (1-dist^2/(dist^2+obj.detectorParameter.GMKernelSigma));
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
       
       function [X,x0,validPoints] = gaussNewton(obj,flagNorm)
           height = size(obj.inputImage,1);
           width = size(obj.inputImage,2);
           validY=((obj.affinePredictions(:,2)>11)&(obj.affinePredictions(:,2)<=height-11));
           validX=((obj.affinePredictions(:,1)>11)&(obj.affinePredictions(:,1)<=width-11));
           validPoints = validX&validY;           
           x0 = [obj.pointsToPredict(validPoints,1)...
               obj.pointsToPredict(validPoints,2)]; 
           %[gx,gy] = imgradientxy(obj.curv_smooth);
           [gx,gy] = imgradientxy(obj.curv);
           
           X_bar = ceil(obj.affinePredictions);
           xt_barx = X_bar(validPoints,1);
           xt_bary = X_bar(validPoints,2);
           X_bar = [xt_barx xt_bary];
           X = X_bar;
           max_iter = 10;
           lambda = 0.0001;
           yita =10;
           %c = 1;%%%% weight against gradient
           sigma = obj.detectorParameter.GMKernelSigma;
           kappa = obj.curv_smooth;
           kappaMax = max(max(kappa));
           for ii = 1:length(X_bar(:,1))
               iter = 0;
               x = X(ii,:);
               %disp(x)
               x_bar = X_bar(ii,:);%(x,y)
               last_res = inf;
               while iter<max_iter %vectorize?????
                  if(flagNorm==2)
                      dist = norm(x-x_bar,2);
                      %dReg = 2/(dist^2+sigma^2)*x;
                      dReg = sigma^2*log(dist^2+sigma^2)*2*x;
                  elseif(flagNorm==1)
                      dist = norm(x-x_bar,1);
                      %dReg = 2/(dist^2+sigma^2)*dist*sign(dist);
                      dReg = sigma^2*log(dist^2+sigma^2)*2*dist*sign(x-x_bar);
                  end
                  dKappa = zeros(1,2);
                  dKappa(1,1)=bi(x(1,2),x(1,1),gx);
                  dKappa(1,2)=bi(x(1,2),x(1,1),gy);
                  %bi(x(1,2)-1,x(1,1)-1,kappa)-bi(x(1,2)+1,x(1,1)+1,kappa)...
                  %    +2*bi(x(1,2),x(1,1)-1,kappa)-2*bi(x(1,2),x(1,1)+1,kappa)...
                  %    +bi(x(1,2)+1,x(1,1)-1,kappa)-bi(x(1,2)+1,x(1,1)+1,kappa);%computing dk/dx
                  %dKappa(1,2)= bi(x(1,2)-1,x(1,1)-1,kappa)-bi(x(1,2)+1,x(1,1)-1,kappa)...
                  %    +2*bi(x(1,2)-1,x(1,1),kappa)-2*bi(x(1,2)+1,x(1,1),kappa)...
                  %    +bi(x(1,2)-1,x(1,1)+1,kappa)-bi(x(1,2)+1,x(1,1)+1,kappa);%computing dk/dy
                  dKappa = yita*dKappa;
                  J = dReg-dKappa/kappaMax;
                  term1 = (kappaMax-bi(x(1,2),x(1,1),kappa))/kappaMax;
                  term2 = dist^2/(dist^2+sigma^2);
                  res = term1+term2;
                  kernel = inv(J'*J+lambda*eye(2));
                  %kernel = (J'*J)^(-1);
                  delta = kernel*J'*res;
                  dres = abs(last_res-res);
                  normDelta = norm(delta);
                  substract = (x'-delta);
                  if ((normDelta<1e-5)||(dres<1e-9)||normDelta>20||(substract(1)<1)||(substract(2)<1)||(substract(1)>=width)||(substract(2)>=height))%&& (res > last_res))
                      %disp('finish opt!!')
                      %x = last_x;
                      break;
                  else
                      x = (x' - delta)';%%%Norm-1 delta ´ó?\
                      last_res = res;
                      %last_x = x;
                      iter = iter +1;
                  end
               end
               X(ii,:) = x;
           end
       end
       
       function [X,x0] = steepestDescent(obj,flagNorm)
           height = size(obj.inputImage,1);
           width = size(obj.inputImage,2);
           validY=((obj.affinePredictions(:,2)>11)&(obj.affinePredictions(:,2)<=height-11));
           validX=((obj.affinePredictions(:,1)>11)&(obj.affinePredictions(:,1)<=width-11));
           validPoints = validX&validY;           
           x0 = [obj.previousLocalMaxima(validPoints,1)...
               obj.previousLocalMaxima(validPoints,2)]; 
           X_bar = ceil(obj.affinePredictions);
           xt_barx = X_bar(validPoints,1);
           xt_bary = X_bar(validPoints,2);
           X_bar = [xt_barx xt_bary];
           X = X_bar;
           [gx,gy] = imgradientxy(obj.curv);
           lr = 1e-5;
           max_iter = 10;
           iter = 0;
           e = 1e-5;
           sigma = obj.detectorParameter.GMKernelSigma;
           kappa = obj.curv;
           kappaMax = max(max(kappa));
           for ii = 1:length(X_bar(:,1))
               iter = 0;
               x = X(ii,:);
               %disp(x)
               x_bar = X_bar(ii,:);%(x,y)
               J = 0;
               x_old = x-1;%initiallize
               while iter<max_iter
                  if(flagNorm==2)
                      dist = norm(x-x_bar,2);
                      %dReg = 2/(dist^2+sigma^2)*x;
                      dReg = -sigma^2*log(dist^2+sigma^2)*2*x;
                  elseif(flagNorm==1)
                      dist = norm(x-x_bar,1);
                      %dReg = 2/(dist^2+sigma^2)*dist*sign(dist);
                      dReg = -sigma^2*log(dist^2+sigma^2)*2*dist*sign(x-x_bar);
                  end
                  dKappa = zeros(1,2);
                  dKappa(1,1)=bi(x(1,2),x(1,1),gx);
                  %ii
                  %iter
                  dKappa(1,2)=bi(x(1,2),x(1,1),gy);
                  %dKappa(1,1)= bi(x(1,2)-1,x(1,1)-1,kappa)-bi(x(1,2)+1,x(1,1)+1,kappa)...
                  %    +2*bi(x(1,2),x(1,1)-1,kappa)-2*bi(x(1,2),x(1,1)+1,kappa)...
                  %    +bi(x(1,2)+1,x(1,1)-1,kappa)-bi(x(1,2)+1,x(1,1)+1,kappa);%computing dk/dx
                  %dKappa(1,2)= bi(x(1,2)-1,x(1,1)-1,kappa)-bi(x(1,2)+1,x(1,1)-1,kappa)...
                  %    +2*bi(x(1,2)-1,x(1,1),kappa)-2*bi(x(1,2)+1,x(1,1),kappa)...
                  %    +bi(x(1,2)-1,x(1,1)+1,kappa)-bi(x(1,2)+1,x(1,1)+1,kappa);%computing dk/dy
                  J_old = J;
                  J = dReg-dKappa/kappaMax;
                  normJ = norm(J);
                  lambda = abs((x-x_old)*(J-J_old)')/norm(J-J_old)^2;
                  if isnan(lambda)
                      break;
                  end
                  x_old = x;
                  x = x-lambda*J;
                  if(normJ<=e)
                      break;
                  end

                  iter = iter +1;
               end
               X(ii,:) = x;
           end
       end
       
       function [X,x0] = LMA(obj,flagNorm)
           height = size(obj.inputImage,1);
           width = size(obj.inputImage,2);
           validY=((obj.affinePredictions(:,2)>11)&(obj.affinePredictions(:,2)<=height-11));
           validX=((obj.affinePredictions(:,1)>11)&(obj.affinePredictions(:,1)<=width-11));
           validPoints = validX&validY;           
           x0 = [obj.previousLocalMaxima(validPoints,1)...
               obj.previousLocalMaxima(validPoints,2)]; 
           [gx,gy] = imgradientxy(obj.curv);
           
           X_bar = ceil(obj.affinePredictions);
           xt_barx = X_bar(validPoints,1);
           xt_bary = X_bar(validPoints,2);
           X_bar = [xt_barx xt_bary];
           X = X_bar;
           max_iter = 10;
           tao = 0.0001;
           %c = 1;%%%% weight against gradient
           sigma = obj.detectorParameter.GMKernelSigma;
           kappa = obj.curv;
           kappaMax = max(max(kappa));
           
           for ii = 1:length(X_bar(:,1))
               iter = 0;
               x = X(ii,:);
               %disp(x)
               x_bar = X_bar(ii,:);%(x,y)
               v=1.1;
               yita =10;
               while iter<max_iter %vectorize?????
                  if(flagNorm==2)
                      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      % HERE TO USE dist_new
                      dist = norm(x-x_bar,2);
                      %dReg = 2/(dist^2+sigma^2)*x;
                      dReg = -sigma^2*log(dist^2+sigma^2)*2*x;
                  elseif(flagNorm==1)
                      dist = norm(x-x_bar,1);
                      %dReg = 2/(dist^2+sigma^2)*dist*sign(dist);
                      dReg = -sigma^2*log(dist^2+sigma^2)*2*dist*sign(x-x_bar);
                  end
                  dKappa = zeros(1,2);

                  dKappa(1,1)=bi(x(1,2),x(1,1),gx);
                  dKappa(1,2)=bi(x(1,2),x(1,1),gy);
                  %dKappa(1,1)= bi(x(1,2)-1,x(1,1)-1,kappa)-bi(x(1,2)+1,x(1,1)+1,kappa)...
                  %    +2*bi(x(1,2),x(1,1)-1,kappa)-2*bi(x(1,2),x(1,1)+1,kappa)...
                  %    +bi(x(1,2)+1,x(1,1)-1,kappa)-bi(x(1,2)+1,x(1,1)+1,kappa);%computing dk/dx
                  %dKappa(1,2)= bi(x(1,2)-1,x(1,1)-1,kappa)-bi(x(1,2)+1,x(1,1)-1,kappa)...
                  %    +2*bi(x(1,2)-1,x(1,1),kappa)-2*bi(x(1,2)+1,x(1,1),kappa)...
                  %    +bi(x(1,2)-1,x(1,1)+1,kappa)-bi(x(1,2)+1,x(1,1)+1,kappa);%computing dk/dy
                  dKappa = dKappa*yita;
                  J = dReg-dKappa/kappaMax;
                  res = (kappaMax-bi(x(1,2),x(1,1),kappa))/kappaMax+dist^2/(dist^2+sigma^2);
                  JTJ = J'*J;
                  if(iter==0)
                      A0=(JTJ);
                      lambda=tao*max(max(A0));
                  else
                      lambda = lambda/v;
                  end
                  kernel = JTJ+lambda*diag(JTJ);
                  kernel_inv = (kernel)^(-1);
                  %kernel = (JTJ+lambda)^(-1);
                  delta = kernel_inv*J'*res;
                  if delta(1)<1e-5&&delta(2)<1e-5
                      break;
                  else
                      x_old = x;
                      x_new = (x' + delta)';%%%Norm-1 delta ´ó?
                      if(flagNorm==2)
                        dist_new = norm(x_new-x,2);
                      elseif(flagNorm==1)
                        dist_new = norm(x_new-x,1);
                      end
                      res_new = (kappaMax-bi(x_new(1,2),x_new(1,1),kappa))/kappaMax+dist_new^2/(dist_new^2+sigma^2);
                      actual = norm(res-res_new);
                      predicted = abs(J*(x_new-x_old)');
                      rho = actual/predicted;
                      if(rho>0.1&&rho<0.25)
                          x = x_new;
                      elseif(rho>0.25)%reduction of residua is rapid
                          x = x_new;
                          v=v*1.1;%increase trust region size v
                      elseif(rho>0&&rho<0.1)
                          x = x_new;
                          v=v/1.1;%reduce trust region size v
                      else
                          v=v/1.1;%reduce trust region size v
                      end
                      iter = iter +1;
                  end
               end
               X(ii,:) = x;
           end
       end
       
       function [orient, loc,inliersXt0,inliersXt1] = poseEstimation(obj)
%            [M,inliersIndex] = estimateEssentialMatrix(obj.xt0,obj.xt1,...
%                obj.detectorParameter.cameraParameter,'Confidence',99,'MaxDistance',1.0,'MaxNumTrials',2000);
            featurePoints_xt0 = obj.matchesInPreviousFrame.Location;
            featurePoints_xt0 = featurePoints_xt0*2; %unsample coordinates
            featurePoints_xt1 = obj.matchesInCurrentFrame.Location;
            featurePoints_xt1 = featurePoints_xt1*2;
            [M,inliersIndex] = estimateEssentialMatrix(featurePoints_xt0,featurePoints_xt1,...
               obj.detectorParameter.cameraParameter,'Confidence',99,'MaxDistance',0.1,'MaxNumTrials',2000);
            EssentialMatrix = M
%            [M,inliersIndex] = estimateFundamentalMatrix(obj.xt0,obj.xt1,...
%                'Method','RANSAC','DistanceThreshold',0.01,'DistanceType','Algebraic');
            inliersXt0 = featurePoints_xt0(inliersIndex,:);
            inliersXt1 = featurePoints_xt1(inliersIndex,:);
            [orient, loc] = relativeCameraPose(M,obj.detectorParameter.cameraParameter,inliersXt0,inliersXt1)
       end
       
       function [worldPoints,reproErrors] = triangulateValidPoints(obj)
           projMatrix = obj.cameraMatrix';%3*4 to 4*3?
           [worldPoints,reproErrors] = triangulate(obj.inliersXt0,obj.inliersXt1,projMatrix,projMatrix);
           %error = reproErrors
       end
   end
end
function r = bi(y,x,image) %bilinear interpolation
x1 = floor(x);
x2 = floor(x)+1;
y1 = floor(y);
y2 = floor(y)+1;
Q11 = image(y1,x1)/((x2-x1)*(y2-y1))*((x2-x)*(y2-y));
Q21 = image(y1,x2)/((x2-x1)*(y2-y1))*((x-x1)*(y2-y));
Q12 = image(y2,x1)/((x2-x1)*(y2-y1))*((x2-x)*(y-y1));
Q22 = image(y2,x2)/((x2-x1)*(y2-y1))*((x-x1)*(y-y1));
r = Q11+Q21+Q12+Q22;
end