%-----------------------------------------------------------------------
% 1-point RANSAC EKF SLAM from a monocular sequence
%-----------------------------------------------------------------------

% Copyright (C) 2010 Javier Civera and J. M. M. Montiel
% Universidad de Zaragoza, Zaragoza, Spain.

% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation. Read http://www.gnu.org/copyleft/gpl.html for details

% If you use this code for academic work, please reference:
%   Javier Civera, Oscar G. Grasa, Andrew J. Davison, J. M. M. Montiel,
%   1-Point RANSAC for EKF Filtering: Application to Real-Time Structure from Motion and Visual Odometry,
%   to appear in Journal of Field Robotics, October 2010.

%-----------------------------------------------------------------------
% Authors:  Javier Civera -- jcivera@unizar.es 
%           J. M. M. Montiel -- josemari@unizar.es

% Robotics, Perception and Real Time Group
% Aragón Institute of Engineering Research (I3A)
% Universidad de Zaragoza, 50018, Zaragoza, Spain
% Date   :  May 2010
%-----------------------------------------------------------------------

function [ filter, features_info,obj,R ] = anj_initialize_a_blob_feature( step, cam, filter, features_info,num_features_to_initialize,obj)

% numerical values
%half_patch_size_when_initialized = 20;
%half_patch_size_when_matching = 6;
%excluded_band = half_patch_size_when_initialized + 1;
%max_initialization_attempts = 1;
%initializing_box_size = [60,40];
%initializing_box_semisize = initializing_box_size/2;
initial_rho = 1;
std_rho = 1;

std_pxl = get_std_z(filter);

%rand_attempt = 1;
%not_empty_box = 1;
%detected_new=0;


features_info = predict_camera_measurements( get_x_k_k(filter), cam, features_info );

uv_pred = [];
for i=1:length(features_info)
    uv_pred = [uv_pred features_info(i).h'];
end
all_uv=[];
%im_size = size(im_k);
%height = im_size(1);
%width=im_size(2);
%region = im_k(21:height-20,21:width-20);
%obj = vitamineDetector(region, 'sobel');
R = [];
if(num_features_to_initialize>=length(obj.currentLocalMaxima(:,1)))
    %R=randi([1,length(obj.xt1(:,1))],num_features_to_initialize,1);
    for i = 1:length(obj.currentLocalMaxima(:,1))
        uv = obj.currentLocalMaxima(i,:)';
        all_uv=[all_uv,uv];
    end
else
    R=randi([1,length(obj.currentLocalMaxima(:,1))],num_features_to_initialize,1);
    for i = 1:length(R)
    	uv = obj.currentLocalMaxima(R(i),:)';
    	all_uv=[all_uv,uv];
    end
end
%nPoints=size(all_uv,2);

%features = obj.currentFeatures;
%descriptors = obj.currentDescriptors;
%all_uv = descriptors.Location';

for i = 1:length(all_uv(1,:))
    uv = double(all_uv(:,i));
    
    if(~isempty(uv))
        
        % add the feature to the filter
        [ X_RES, P_RES, newFeature ] = add_features_inverse_depth( uv, get_x_k_k(filter),...
            get_p_k_k(filter), cam, std_pxl, initial_rho, std_rho );
        filter = set_x_k_k(filter, X_RES);
        filter = set_p_k_k(filter, P_RES);
        
        % add the feature to the features_info vector
        features_info = anj_add_feature_to_info_vector( uv, X_RES, features_info, step, newFeature);
        %feature here is not correct, should be a single feature
        
    end
    
end
    for i=1:length(features_info)
        features_info(i).h = [];
    end
