function [ filter, features_info, uv ] = anj_init_features(step, cam, im_k, filter, features_info,obj)
std_pxl = get_std_z(filter);
initial_rho = 1;
std_rho = 1;
for i=1:size(obj.location,1)
    features_info = predict_camera_measurements( get_x_k_k(filter), cam, features_info );
    % add the feature to the filter
    [ X_RES, P_RES, newFeature ] = add_features_inverse_depth( obj.location(i,:)', get_x_k_k(filter),...
            get_p_k_k(filter), cam, std_pxl, initial_rho, std_rho );
    filter = set_x_k_k(filter, X_RES);
    filter = set_p_k_k(filter, P_RES);
            
    % add the feature to the features_info vector
    features_info = add_feature_to_info_vector( obj.location(i,:)', im_k, X_RES, features_info, step, newFeature );
end

end