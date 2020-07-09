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

clear variables; close all; clc;
rand('state',0); % rand('state',sum(100*clock));
%wr = VideoWriter('result_vitamine.avi');
%open(wr);
%-----------------------------------------------------------------------
% Sequence, camera and filter tuning parameters, variable initialization
%-----------------------------------------------------------------------

% Camera calibration
cam = initialize_cam;

% Set plot windows
set_plots;

% Sequence path and initial image
sequencePath = 'D:\workspace\data\dataset\sequences\00\image_0\';
initIm = 0;
lastIm = 1000;

% Initialize state vector and covariance
[x_k_k, p_k_k] = initialize_x_and_p;

% Initialize EKF filter
sigma_a = 0.007; % standar deviation for linear acceleration noise
sigma_alpha = 0.007; % standar deviation for angular acceleration noise
sigma_image_noise = 1.0; % standar deviation for measurement noise
filter = ekf_filter( x_k_k, p_k_k, sigma_a, sigma_alpha, sigma_image_noise, 'constant_velocity' );

% variables initialization
features_info = [];
trajectory = zeros( 7, lastIm - initIm );
% other
max_number_of_features_in_image = 250;
generate_random_6D_sphere;
measurements = []; predicted_measurements = [];

%---------------------------------------------------------------
% Main loop
%---------------------------------------------------------------

im = takeImage( sequencePath, initIm );
im_size = size(im);
height = im_size(1);
width=im_size(2);
region = im(21:height-20,21:width-20);
obj = vitamineDetector(region,'sobel');
%means_toplot = [];
for step=initIm+1:lastIm
    
    % Map management (adding and deleting features; and converting inverse depth to Euclidean)
    %[ filter, features_info ] = anj_feature_management( filter, features_info, cam, im, min_number_of_features_in_image, step );
    [ filter, features_info,obj,R ] = map_management( filter, features_info, cam, max_number_of_features_in_image, step,obj );

    % EKF prediction (state and measurement prediction)
    [ filter, features_info ] = ekf_prediction( filter, features_info );
    
    % Grab image
    im = takeImage( sequencePath, step );
    
    % Search for individually compatible matches
    features_info = search_IC_matches( filter, features_info, cam, im );
    
    %Update the detector here? matches are established in this step
    %update the match information into features_info
    %[features_info,obj,means,points] = vitamineMatching(im,obj,features_info,R);
    [features_info,obj,points] = vitamineMatching(im,obj,features_info,R);
    %means_toplot = [means_toplot;means];
    % 1-Point RANSAC hypothesis and selection of low-innovation inliers
    features_info = ransac_hypotheses( filter, features_info, cam );
    
    % Partial update using low-innovation inliers
    filter = ekf_update_li_inliers( filter, features_info );
    
    % "Rescue" high-innovation inliers
    features_info = rescue_hi_inliers( filter, features_info, cam );
    
    % Partial update using high-innovation inliers
    filter = ekf_update_hi_inliers( filter, features_info );

    % Plots,
    plots; display( step );
    
    % Save images
    %saveas( figure_all, sprintf( '%s/image%04d.fig', 'result_S_test1', step ), 'fig' );
    %frame=getframe(gcf);
    %wr = addframe( movie, im );
    %writeVideo(wr,frame);

end
%close(wr);
% Mount a video from saved Matlab figures
%fig2avi;