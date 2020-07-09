%%% bundle adjustment
if(obj.ifInitialized==1&&step~=1)
    trackPoints = findTracks(obj.vSet);
%     Orientation = obj.orientation';
%     Location = obj.location';
%     ViewId = uint32(obj.ViewID');
%     posesToOptim = table(ViewId,Orientation,Location);
    posesToOptim = poses(obj.vSet);
    [worldPoints,reprojectionError] = triangulateMultiview(trackPoints,posesToOptim,camera.Intrinsics);
    idx = reprojectionError<1;
    validWorldPoints = worldPoints(idx,:);
    validTrackPoints = trackPoints(idx);
%     [validWorldPoints,refinedPoses] = bundleAdjustment(validWorldPoints,validTrackPoints,...
%         posesToOptim,camera.Intrinsics,'MaxIterations',200,'AbsoluteTolerance',1e-12,...
%         'RelativeTolerance',1e-12,'PointsUndistorted',true,'FixedViewID',1);
    refinedPoses = posesToOptim;
%     last_orientation_cell = refinedPoses.Orientation(end);
%     last_orientation = last_orientation_cell{1};
%     last_location_cell = refinedPoses.Location(end);
%     last_location = last_location_cell{1};
    last_location = refinedPoses.AbsolutePose(end).Translation;
    last_orientation = refinedPoses.AbsolutePose(end).Rotation;
    obj = obj.updateCamera(refinedPoses,step);
    obj = obj.updateWorldPoints(validWorldPoints);
%     figure(9);
%     ptCloud = pointCloud(validWorldPoints);
%     hold on;
%     grid on;
%     pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%     'MarkerSize', 45);
%     for jj=1:height(refinedPoses(:,1))
%         gt_matrix = reshape(gt(jj,:),4,3)';
%         gt_location = gt_matrix(1:3,4)';
%         gt_orientation = gt_matrix(1:3,1:3);
%         plotCamera('Location', refinedPoses.AbsolutePose(jj).Translation, 'Orientation', refinedPoses.AbsolutePose(jj).Rotation, 'Size', 0.1, ...
%     'Color', 'b', 'Label', num2str(jj), 'Opacity', 0);
% %         plotCamera('Location',gt_location , 'Orientation',gt_orientation , 'Size', 0.1, ...
% %     'Color', 'r', 'Label', num2str(jj), 'Opacity', 0);
%     end
%     hold off;
else
    last_location = obj.proposed_location;
    last_orientation = obj.proposed_orientation;
    posesToOptim = poses(obj.vSet);
    obj = obj.updateCamera(posesToOptim,step);
end