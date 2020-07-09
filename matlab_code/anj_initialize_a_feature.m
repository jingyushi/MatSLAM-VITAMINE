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

function [ filter, features_info, no_features ] = anj_initialize_a_feature( step, cam, im_k, filter, features_info )

% numerical values
half_patch_size_when_initialized = 20;
initial_rho = 1;
std_rho = 1;

std_pxl = get_std_z(filter);
features_info = predict_camera_measurements( get_x_k_k(filter), cam, features_info );

uv_pred = [];
features=[];
for i=1:length(features_info)
    uv_pred = [uv_pred features_info(i).h'];
    features=[features;features_info(i).feature];
end

obj = blob(im_k, 'SURF');

    for i = 1:size(obj.descriptors)
        uv=obj.location(i,:)';
            if((uv(2,1)- half_patch_size_when_initialized)>0 && (uv(2,1)+half_patch_size_when_initialized)<=cam.nRows && ...
                (uv(1,1)-half_patch_size_when_initialized)>0 && (uv(1,1)+half_patch_size_when_initialized)<=cam.nCols)
                [ X_RES, P_RES, newFeature ] = add_features_inverse_depth( uv, get_x_k_k(filter),...
                get_p_k_k(filter), cam, std_pxl, initial_rho, std_rho );
                filter = set_x_k_k(filter, X_RES);
                filter = set_p_k_k(filter, P_RES);
                % add the feature to the features_info vector
                features_info = anj_add_feature_to_info_vector( uv, im_k, X_RES, features_info, step, newFeature,obj.features(i,:),obj.descriptors(i));
            end 
    
    end

    
for i=1:length(features_info)
    features_info(i).h = [];
end
no_features = size(features_info,2);
end