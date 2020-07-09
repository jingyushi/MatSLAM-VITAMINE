%-----------------------------------------------------------------------
% Authors:  Anjali Dhabaria -- adhabaria3@gatech.edu
%-----------------------------------------------------------------------

function features_info = anj_matching( im, features_info, cam )
obj = blob(im, 'SURF');
for i_feature=1:length(features_info) % for every feature in the map
    
    if ~isempty(features_info(i_feature).h); % if it is predicted, search in the region
        
        h = features_info(i_feature).h;
        S = features_info(i_feature).S;
        
        if eig(S)< 100 %if the ellipse is too big, do not search (something may be wrong)
            indexPairs = matchFeatures(features_info(i_feature).feature,obj.features);
            %features_info(obj.index_pairs(i,1)).z = obj.previousDescriptors.Location(obj.index_pairs(i,1));
            if( ~isempty(indexPairs))
                features_info(i_feature).individually_compatible = 1;
                features_info(i_feature).z = double(obj.descriptors(indexPairs(1,2)).Location');
            end
            
        end
        
    end
    
end