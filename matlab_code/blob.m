classdef blob
   properties
       detectorParameter = struct(...
           'descriptorType','SURF',...
           'sigma', 2,...
           'num_scales', 10,...
           'scaleMultiplier', sqrt(sqrt(2)),...
           'threshold', 0.0095);
       %%descriptor type coud be SURF or FREAK
       
       h; w; %sigma; num_scales; scaleMultiplier; threshold;
       scale_space; nms_3d; location;
       %wr;
       index_pairs;
       inputImage;
       previousImage;
       descriptors;
       features;
       previousDescriptors;
       previousFeatures;
       matchesInPreviousFrame;
       matchesInCurrentFrame;
       
   end
   methods
       function obj = blob(inputImage, descriptorType)
          %obj.wr = VideoWriter('result_surf.avi');
          %open(obj.wr);
          obj.detectorParameter.descriptorType = descriptorType;
          if ndims(inputImage)==3
               obj.inputImage = rgb2gray(inputImage); % assume that the input is a grayscale image
          else
               obj.inputImage = inputImage;
          end
          obj.inputImage = im2double(obj.inputImage);
          %%Intializations
          [obj.h, obj.w] = size(obj.inputImage); 
          %obj.detectorParameter.num_scales = 10; obj.detectorParameter.sigma = 2; obj.detectorParameter.scaleMultiplier = sqrt(sqrt(2)); obj.detectorParameter.threshold = 0.0095;
          
          obj.scale_space = obj.generateScaleSpace();
          obj.nms_3d = obj.nms();
          %obj = obj.calcRadiiByScale();
          obj.location = obj.retrieveBlobMarkers(); 
          %radii = obj.blobMarkers(:,3); %radii
          %obj.location = obj.blobMarkers(:,1:2);
          [obj.features,obj.descriptors] = obj.computeDescriptor();
          %xPos = blobMarkers(:,1); %col positions
          %yPos = blobMarkers(:,2); %row positions

          
       end
       function obj = updateDetector(obj,inputImage)
           obj.previousDescriptors = obj.descriptors;
           obj.previousFeatures = obj.features;
           obj.previousImage = obj.inputImage;
           if ndims(inputImage)==3
               obj.inputImage = rgb2gray(inputImage); % assume that the input is a grayscale image
           else
               obj.inputImage = inputImage;
           end
           obj.inputImage = im2double(obj.inputImage);
           obj.scale_space = obj.generateScaleSpace();
           obj.nms_3d = obj.nms();
           obj.location = obj.retrieveBlobMarkers(); 
           [obj.features,obj.descriptors] = obj.computeDescriptor();
           [obj.matchesInPreviousFrame,obj.matchesInCurrentFrame, obj.index_pairs]=obj.matcher();
           %obj.location = [x_pos, y_pos];

       end
       function scale_space = generateScaleSpace(obj)
           scale_space = zeros(obj.h,obj.w,obj.detectorParameter.num_scales); % [h,w] - dimensions of image, n - number of levels in scale space
           kernelSize = max(1,fix(6*obj.detectorParameter.sigma)+1);
           LoGKernel = obj.detectorParameter.sigma^2 * fspecial( 'log', kernelSize, obj.detectorParameter.sigma);
           for i=1:obj.detectorParameter.num_scales
               % Downsize the image by 1/k... use bicubic instead of bilinear to
               % keep spatial resolution. Here k is the scaleMultiplicationConstant
               if i==1
                   downsizedImg = obj.inputImage;
               else
                   downsizedImg = imresize(obj.inputImage, 1/(obj.detectorParameter.scaleMultiplier^(i-1)), 'bicubic');
               end
               filteredImage = imfilter(downsizedImg, LoGKernel,'same', 'replicate');
               %Save square of Laplacian response for current level of scale space
               filteredImage = filteredImage .*filteredImage;
               reUpscaledImg = imresize(filteredImage, [obj.h,obj.w], 'bicubic');
               scale_space(:,:,i) = reUpscaledImg;
           end
           
       end
       function nms_3d = nms(obj)
           nms_2d = zeros(obj.h,obj.w,obj.detectorParameter.num_scales);
           neighborhoodSize = 3; %size of mask
           domain = ones(3,3);           
           for i = 1:obj.detectorParameter.num_scales
               %%Maximum filter;
               nms_2d(:,:,i) = ordfilt2(obj.scale_space(:,:,i), neighborhoodSize^2, domain);
           end
           maxVals_InNeighboringScaleSpace = nms_2d;
           for i = 1:obj.detectorParameter.num_scales
               if i == 1
                   lowerScale = i;upperScale = i+1;
               elseif i < obj.detectorParameter.num_scales
                   lowerScale = i-1;upperScale = i+1;
               else
                   lowerScale = i-1;upperScale = i;
               end
               %each row and column holds the maximum value at that row and col from 
               %the neighboring scale space... 
               %ie: the maximum of the value at pix x,y in the scale space above, 
               %current and below... so neighboring scales will end up with the same
               %values at many row and col positions... this is an intermediate calc
               maxVals_InNeighboringScaleSpace(:,:,i) = max(maxVals_InNeighboringScaleSpace(:,:,lowerScale:upperScale),[],3);
           end

           %mark every location where the max value is the actualy value from that
           %scale with a 1, and a 0 otherwise. (Binary flag)
           originalValMarkers = maxVals_InNeighboringScaleSpace == obj.scale_space;
           %only keep the max vals that were actually from those locations
           nms_3d = maxVals_InNeighboringScaleSpace .* originalValMarkers;
           threshBinaryFlag = nms_3d > obj.detectorParameter.threshold;
           nms_3d = nms_3d .* threshBinaryFlag;
       end
    
       function [radiiByScale] = calcRadiiByScale(obj)
           radiiByScale = zeros(1,obj.detectorParameter.num_scales);
           for i = 1:obj.detectorParameter.num_scales
               radiiByScale(i) =  sqrt(2) * obj.detectorParameter.sigma * obj.detectorParameter.scaleMultiplier^(i-1); 
           end
       end

       function [a,b] = computeDescriptor(obj)
           %Blob Detector
           points=SURFPoints(obj.location);
           points = selectStrongest(points,5);   
           %points = BRISKPoints(obj.location);
           [a,b] = extractFeatures(obj.inputImage,points,'Method',obj.detectorParameter.descriptorType);
           %figure; imshow(obj.inputImage); hold on;
           %plot(b,'showOrientation',true);
       end

       function [blobM] = retrieveBlobMarkers(obj)
           blobM = [];
           for i = 1:obj.detectorParameter.num_scales
               [newMarkerRows, newMarkerCols] = find(obj.nms_3d(:,:,i));
               newMarkers = [newMarkerCols'; newMarkerRows'];
               %newMarkers(3,:) = obj.radiiByScale(i);
               blobM = [blobM; newMarkers'];  
           end
       end
       function [a,b,indexPairs] = matcher(obj)
           indexPairs = matchFeatures(obj.previousFeatures,obj.features, 'MatchThreshold', 60);
           a = obj.previousDescriptors(indexPairs(:,1));
           b = obj.descriptors(indexPairs(:,2));
           %figure; showMatchedFeatures(obj.previousImage,obj.inputImage,a,b);
           figure; ax = axes; showMatchedFeatures(obj.previousImage,obj.inputImage,a,b, 'montage','Parent',ax);
           %frame=getframe(gcf);
           %writeVideo(obj.wr,frame);
       end
   end
end