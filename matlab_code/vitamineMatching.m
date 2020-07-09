function [features_info,obj,points] = vitamineMatching(im,obj,features_info,R)
    %obj = obj.updateDetector(im);
    points=[];
    for ii = 1:length(features_info)
        points = [points;features_info(ii).uv_when_initialized];%[x,y]
    end
    obj = obj.pointsPrediction(points,im);
    matchedList = obj.validPoints;%%length of original local maxima
    xt1 = obj.xt1;%[x,y]
    %**************************
    
    %**************************
%     mins_gn = zeros(length(xt1(:,1)),2);
%     for ii =1:length(xt1(:,1))
%         mindist = inf;
%         minid = -1;
%         for jj = 1:length(obj.currentLocalMaxima(:,1))
%             dist = norm(xt1(ii,:)-obj.currentLocalMaxima(jj,:),1);
%             if dist<mindist
%                 mindist = dist;
%                 minid = jj;
%             end
%         end
%         mins_gn(ii,1)=mindist;
%         mins_gn(ii,2)=minid;
%     end
%     meandist_gaussnewton = mean(mins_gn(:,1));
%     %figure(2);hold on;
%     xx_gn = 1:length(xt1(:,1));
%     %plot(xx,mins_gn(:,1),'b');
%     mins_hc = zeros(length(obj.xt1_new(:,1)),2);
%     for ii =1:length(obj.xt1_new(:,1))
%         mindist = inf;
%         minid = -1;
%         for jj = 1:length(obj.currentLocalMaxima(:,1))
%             dist = norm(obj.xt1_new(ii,:)-obj.currentLocalMaxima(jj,:),1);
%             if dist<mindist
%                 mindist = dist;
%                 minid = jj;
%             end
%         end
%         mins_hc(ii,1)=mindist;
%         mins_hc(ii,2)=minid;
%     end
%     meandist_hillclimbing = mean(mins_hc(:,1));
%     xx_hc = 1:length(xt1(:,1));
%     %plot(xx,mins_hc(:,1),'r');
%     %**************************
%     mins = zeros(length(obj.affinePredictions(:,1)),2);
%     for ii =1:length(obj.affinePredictions(:,1))
%         mindist = inf;
%         minid = -1;
%         for jj = 1:length(obj.currentLocalMaxima(:,1))
%             dist = norm(obj.affinePredictions(ii,:)-obj.currentLocalMaxima(jj,:),1);
%             if dist<mindist
%                 mindist = dist;
%                 minid = jj;
%             end
%         end
%         mins(ii,1)=mindist;
%         mins(ii,2)=minid;
%     end
%     meandist_affine = mean(mins(:,1));
% %     xx_a = 1:length(obj.affinePredictions(:,1));
% %     plot(xx_a,mins(:,1),'k--',xx_hc,mins_hc(:,1),'r',xx_gn,mins_gn(:,1),'g');
%     %**************************
% %     figure(3);
% %     hold on;
% %     imagesc(im);
% %     colormap gray;
% %     for ii =1:length(obj.currentLocalMaxima(:,1))
% %         plot(obj.currentLocalMaxima(ii,1),obj.currentLocalMaxima(ii,2),'g+','Markersize',8);
% %     end
% %     for ii = 1:length(xt1(:,1))
% %         plot(xt1(ii,1),xt1(ii,2),'r+','Markersize',8);
% %     end
% %     for ii = 1:length(obj.affinePredictions(:,1))
% %         if(obj.validPoints(ii))
% %             plot(obj.affinePredictions(ii,1),obj.affinePredictions(ii,2),'b+','Markersize',8);
% %             plot([obj.affinePredictions(ii,1),obj.pointsToPredict(ii,1)],...
% %                 [obj.affinePredictions(ii,2),obj.pointsToPredict(ii,2)],'Color','green','LineWidth',1);
% %         end
% %     end
% %     hold off;
%     %**************************
%     means = [meandist_gaussnewton,meandist_hillclimbing,meandist_affine];
    matchCount = 0;
    for ii = 1:length(features_info)
        if(matchedList(ii))
            matchCount = matchCount+1;
            features_info(ii).z = xt1(matchCount,:)';%[x,y]
            features_info(ii).individually_compatible = 1;
        end
    end
%     matchCount;
%     xt1_all = zeros(length(matchedList),2);
%     for ii=1:length(matchedList)
%         if(matchedList(ii))
%             matchCount = matchCount+1;
%             xt1_all(ii,:) = xt1(matchCount,:);
%         end
%     end
%     matchCount = 0;
%     if isempty(R)%%%entire set of local maxima has been added in to features_info
%         for ii=length(matchedList):-1:1 
%             jj = length(matchedList)-ii+1;
%             if(matchedList(jj))
%                 matchCount = matchCount+1;
%                 features_info(end-ii+1).z = xt1(matchCount,:)';
%                 features_info(end-ii+1).individually_compatible = 1;
%             end
%         end
%     elseif(R~=-1) %%%%%%%%only a random subset of local maxima has been added into features_info
%         for ii=length(R):-1:1
%             if(matchedList(R(ii)))
%                 features_info(end-ii+1).z = xt1_all(R(ii),:)';%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 features_info(end-ii+1).individually_compatible = 1;
%                 matchCount = matchCount+1;
%             end
%         end
%     else
% %         for ii=length(matchedList):-1:1
% %             if(matchedList(ii))
% %                 
% %                 features_info(end-matchCount).z = xt1(end-matchCount,:)';
% %                 features_info(end-matchCount).individually_compatible = 1;
% %                 matchCount = matchCount+1;
% %             end
% %         end
% %     end
%     end
end