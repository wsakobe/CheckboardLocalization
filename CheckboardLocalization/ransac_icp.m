% This function returns the best rotation matrix, best translation vector and 
% inliers calculated by using estimateWorldCameraPose function with
% randomly selecting points for some number of iterations.

% matches       : contains the indexes of matched descriptors of sift features
%                 3d and sift features extracted from an image

% f_image       : locations sift features extracted from an image

% sift_locations: 3d locations of all sift features of box extracted from
%                 the init_texture folder.

% num_points    : number of points given to estimateWorldCameraPose
%                 function in matlab

% iter          : maximum the number of iterations
% threshold     : the threshold of the distances between actual points and 
%                 the projected point

% inlierRatio   : the minimum ratio of number of inliers to the total
%                 mached features


function [bestInlierNum, bestInlierRatio  , best_R , best_T ] = ransac_icp(camera_locations, world_locations, threshold, num_points, iterations ,inlierRatio)
     
     bestInlierNum = 0;  %Number of points out of num_points which gave the least reprojection error 
     bestInlierRatio = 0; %Number of best inliers/total number of points used by PNP
     best_R = [];
     best_T = [];
%     P = camera_locations';
%     Q = world_locations';
     sift_feature_matches = 1:size(camera_locations,2);
     for i=1:iterations
         % Randomly select 'num_points' number of points and call
         % estimateWorldCameraPose, these points are used by pnp to
         % estimate camera pose 
         idx = randperm(size(sift_feature_matches,2),num_points);
         sample_index = sift_feature_matches(:,idx);
         sample_2d = camera_locations(:,sample_index(1,:));
         sample_3d = world_locations(:,sample_index(1,:));
         data = struct('imagePoints',sample_2d', 'worldPoints',sample_3d');
         %% test without reprojectionerror tbd
         
         try
             [R,T] = SVDsolveRT(data.imagePoints', data.worldPoints');
         catch
             %disp('Error found');
             continue;
         end
         
         % reproject the 3d matched points on the image and calculate the
         % reprojection error
         world_to_camera = R*world_locations + repmat(T, [1 size(sift_feature_matches, 2)]);
         v = world_to_camera - camera_locations;
         vec = [];
         for j=1:size(sift_feature_matches,2)
             vec(:,j) = norm(v(:,j));
         end
         
         % Compute the inliers with distances smaller than the threshold
         inlierIdx = find(abs(vec)<=threshold);
         inlierNum = length(inlierIdx);
         
         % Update the number of inliers and fitting model if better model is found
         if (inlierNum>=round(inlierRatio*size(sift_feature_matches, 2))) && (inlierNum>bestInlierNum)
             bestInlierNum = inlierNum;
             best_R = R;
             best_T = T;
             bestInlierRatio = bestInlierNum/size(sift_feature_matches, 2);
%              fprintf('Best Inlier Number: %d \n',bestInlierNum);
             fprintf('Minimized distance: %.2f \n',mean(vec));
         end
    end 
end