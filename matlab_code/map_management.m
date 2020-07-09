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

function [ filter, features_info,obj,R ] = map_management( filter,...
    features_info, cam,  max_number_of_features_in_image, step,obj )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% delete features, if necessary
[ filter, features_info ] = delete_features( filter, features_info );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

measured = 0;
for i=1:length(features_info)
    %if (features_info(i).low_innovation_inlier || features_info(i).high_innovation_inlier) measured = measured + 1; end
    if (features_info(i).individually_compatible) measured = measured + 1; end
end

% update features info
features_info = update_features_info( features_info );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% convert features from inverse depth to cartesian, if necessary
%[ filter, features_info ] = inversedepth_2_cartesian( filter, features_info );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialize features randomly
if measured >=max_number_of_features_in_image
    R = -1;
    return;
else
    num_features_to_initial = max_number_of_features_in_image-measured;
    [ filter, features_info,obj,R ] = initialize_features( step, cam,...
            filter, features_info, num_features_to_initial,obj);
end
% if measured == 0
%     [ filter, features_info,obj ] = initialize_features( step, cam,...
%         filter, features_info, min_number_of_features_in_image,obj);
% else
%     if measured < min_number_of_features_in_image
%         [ filter, features_info,obj ] = initialize_features( step, cam,...
%             filter, features_info, min_number_of_features_in_image - measured,obj);
%     end
% end