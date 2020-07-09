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
% Arag�n Institute of Engineering Research (I3A)
% Universidad de Zaragoza, 50018, Zaragoza, Spain
% Date   :  May 2010
%-----------------------------------------------------------------------

chi_095_2 = 5.9915;
% chi_099_2 = 9.2103;
chi_095_3 = 7.8147;
% chi_099_3 = 11.3449;

% plot image stuff
figure(figure_all);
subplot(im_fig);
hold off;
imagesc(im);
colormap gray;
hold on;
title('Thick red: low innovation inliers. Thin red: high innovation inliers. \newline Magenta: rejected by 1-point RANSAC. Blue: No match found by cross-correlation');

predicted_index = 0;
predicted_measurements = get_predicted_measurements(filter);
S_predicted = get_S_predicted(filter);

for i=1:length(features_info)

    if (~isempty(features_info(i).h)&&~isempty(features_info(i).S))

        %imagesc( features_info(i).h(1) - features_info(i).half_patch_size_when_matching,...
        %    features_info(i).h(2) - features_info(i).half_patch_size_when_matching, features_info(i).patch_when_matching);

%         if features_info(i).low_innovation_inlier
%             %plotUncertainEllip2D( features_info(i).S,...
%             %    features_info(i).h, chi_095_2, 'r', 2 )
%             plot( features_info(i).h(1), features_info(i).h(2),'r+','Markersize',4);
%         end
%         if features_info(i).high_innovation_inlier
%             %plotUncertainEllip2D( features_info(i).S,...
%             %    features_info(i).h, chi_095_2, 'r', 1 )
%             plot( features_info(i).h(1), features_info(i).h(2),'r+','Markersize',4);
%         end
        if (features_info(i).individually_compatible)&&(features_info(i).high_innovation_inlier==0)&&(features_info(i).low_innovation_inlier==0)
            %plotUncertainEllip2D( features_info(i).S,...
            %    features_info(i).h, chi_095_2, 'm', 1 )
            plot( features_info(i).h(1), features_info(i).h(2),'m+','Markersize',4);
        end
        
%         if ~features_info(i).individually_compatible
%             %plotUncertainEllip2D( features_info(i).S,...
%             %    features_info(i).h, chi_095_2, 'b', 1 )
%             plot( features_info(i).h(1), features_info(i).h(2),'b+','Markersize',4);
%         end
        
        if features_info(i).individually_compatible
            plot( features_info(i).z(1), features_info(i).z(2),'g+','Markersize',4);
            plot([features_info(i).uv_when_initialized(1),features_info(i).z(1)],...
                [features_info(i).uv_when_initialized(2),features_info(i).z(2)],'Color','green','LineWidth',1);
        end

    end

end

% plot predicted and measured
% which_are_predicted = predicted_measurements(:,1)>0;
% plot( predicted_measurements(which_are_predicted,1), predicted_measurements(which_are_predicted,2),'r+','Markersize',10);
% which_are_measured = measurements(:,1)>0;
% plot( measurements(which_are_measured,1), measurements(which_are_measured,2),'g+');

axes_handler = get(gcf,'CurrentAxes');
set(axes_handler,'XTick',[],'YTick',[]);

% plot 3D stuff
figure(figure_all);
subplot(near3D_fig);
hold off;
% 
x_k_k = get_x_k_k(filter);
p_k_k = get_p_k_k(filter);
% 
draw_camera( [x_k_k(1:3); x_k_k(4:7)], 'k' );
hold on;
% imagesc(obj.curv_smooth);
% colorbar;
% hold on;
% title('Green circle: points to be predicted \newline Red square: Affine Prediction \newline Blue diamond: Corrected prediction \newline Yellow cross: Local maxima');
% for ii =1:length(points(:,1))
%     plot(points(ii,1),points(ii,2),'g','Marker','o');
%     plot(obj.affinePredictions(ii,1),obj.affinePredictions(ii,2),'r','Marker','s');
%     plot([points(ii,1),obj.affinePredictions(ii,1)],...
%                 [points(ii,2),obj.affinePredictions(ii,2)],'Color','green','LineWidth',1);
% end
% matchCount = 0;
% for ii = 1:length(obj.affinePredictions(:,1))
%     if(obj.validPoints(ii))
%         matchCount = matchCount+1;
%         plot([obj.affinePredictions(ii,1),obj.xt1(matchCount,1)],...
%                 [obj.affinePredictions(ii,2),obj.xt1(matchCount,2)],'Color','red','LineWidth',1);
%         plot([points(ii,1),obj.xt1(matchCount,1)],...
%                 [points(ii,2),obj.xt1(matchCount,2)],'Color','blue','LineWidth',1);
%         plot(obj.xt1(matchCount,1),obj.xt1(matchCount,2),'b','Marker','d');
%     end
% end
% for ii = 1:length(obj.currentLocalMaxima(:,1))
%     plot(obj.currentLocalMaxima(ii,1),obj.currentLocalMaxima(ii,2),'y','Marker','x');
% end

title('Camera motion and scene in top view [m]. \newline Red arrows stand for point features in infinite depth uncertainty.');

trajectory(:,step - initIm) = x_k_k(1:7);
plot3( trajectory(1, 1:step - initIm), trajectory(2, 1:step - initIm),...
    trajectory(3, 1:step - initIm), 'k', 'LineWidth', 2 );

x_k_k_features_plots = x_k_k(14:end);
p_k_k_features_plots = p_k_k(14:end,14:end);

for i=1:length(features_info)

    if strcmp(features_info(i).type, 'cartesian')
        XYZ = x_k_k_features_plots(1:3);
        p_XYZ = p_k_k_features_plots(1:3,1:3);
        x_k_k_features_plots = x_k_k_features_plots(4:end);
        p_k_k_features_plots = p_k_k_features_plots(4:end,4:end);
        plotUncertainEllip3D(  p_XYZ, XYZ, chi_095_3, 'r', 1  );
        plot3(XYZ(1),XYZ(2),XYZ(3),'r+','Markersize',10)
    end
    if strcmp(features_info(i).type, 'inversedepth')
        y_id = x_k_k_features_plots(1:6);
        XYZ = inversedepth2cartesian( y_id );
        p_id = p_k_k_features_plots(1:6,1:6);
        x_k_k_features_plots = x_k_k_features_plots(7:end);
        p_k_k_features_plots = p_k_k_features_plots(7:end,7:end);
        if y_id(6)-3*sqrt(p_id(6,6))<0
            if ( y_id(6)>0)
                ray = 8*m(y_id(4),y_id(5));
                minimum_distance = inversedepth2cartesian([y_id(1:5); y_id(6)+3*sqrt(p_id(6,6))]);
                %vectarrow([minimum_distance(1) 0 minimum_distance(3)],[ray(1) 0 ray(3)],'r')
                plot3(XYZ(1),XYZ(2),XYZ(3),'r.','Markersize',1)
            end
        else
            if ( y_id(6)>0)
                %plotUncertainSurfaceXZ( p_id, y_id, 0, [1 0 0], randSphere6D, nPointsRand );
                plot3(XYZ(1),XYZ(2),XYZ(3),'r.','Markersize',1)
            end
        end

    end

end


axes_handler = get(gcf,'CurrentAxes');
axis([-4 4 -1 1 -1 7]);
grid on;
view(-360,0);
hold off;
% subplot(im_fig2);
% hold off;
% imagesc(im);
% colormap gray;
% hold on;
% title('Green lines: Affine Transformation \newline Red lines: Correction to the maxima \newline Blue lines: previous points to the correction ');
% for ii =1:length(points(:,1))
%     plot([points(ii,1),obj.affinePredictions(ii,1)],...
%                 [points(ii,2),obj.affinePredictions(ii,2)],'Color','green','LineWidth',1);
% end
% matchCount = 0;
% for ii = 1:length(obj.affinePredictions(:,1))
%     if(obj.validPoints(ii))
%         matchCount = matchCount+1;
%         plot([obj.affinePredictions(ii,1),obj.xt1(matchCount,1)],...
%                 [obj.affinePredictions(ii,2),obj.xt1(matchCount,2)],'Color','red','LineWidth',1);
%         plot([points(ii,1),obj.xt1(matchCount,1)],...
%                 [points(ii,2),obj.xt1(matchCount,2)],'Color','blue','LineWidth',1);
%     end
% end
% 
% axes_handler = get(gcf,'CurrentAxes');
% set(axes_handler,'XTick',[],'YTick',[]);
% 
% subplot(dist_stat);
% title('Mean Hammming distance from \newline predictions to their nearest local maxima');
% hold off;
% meandist_gaussnewton = means_toplot(:,1);
% meandist_hillclimbing = means_toplot(:,2);
% meandist_affine = means_toplot(:,3);
% if((step-initIm)<=100)
% xx = 1:(step-initIm);
% else
%     xx = (step-initIm)-99:(step-initIm);
% end
% plot(xx,meandist_gaussnewton(xx),'r',xx,meandist_hillclimbing(xx),'g',xx,meandist_affine(xx),'k');
% legend('Gauss Newton','Hill Climbing','Only affine model prediction');
% xlabel('step');
% ylabel('mean hamming distance');
% 
% hold on;