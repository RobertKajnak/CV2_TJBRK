function [p,pi,d1,d2] = InterestPoints(f1, f2, matches, n, show_matched_points,im1,im2)
% Calculates the fundamental matrix A(nx9) for the transformation between 
% images im1 and im2
% SHOW_MATCHED_POINTS - display a figure with the points used for the
%                       construction of the matrix
% Specify n<0 to skip ransac and return all possibly matching points
% returned by vl_sift
%
% See also: MATCHED_POINTS, RANSAC, VL_SIFT

    if nargin<3
        n=50;
    end

    if nargin<4
        show_matched_points = 0;
    end

    %% Get matches
    %Finds the keypoints from the two images using vl_swift
    %[~, matches,f1,f2,d1,d2] = keypoint_matching(im1, im2);


    %% Calculate Ransac
    if n>0
        iterations = 10;
        samples = 20; 

        % The transformation matrix m and t are not necessary for this
        % implementation, therefore they are ignored
        [~, best_inliers] = RANSAC(im1,im2,matches, f1, f2, iterations, samples);
        %After calculating the best inliers (manhattan_dist<10), they are sorted
        [~,idx] = sort(best_inliers(3,:));
        %the indices from f for the best n pairs are loaded into matched_points
        n=min(n,size(best_inliers,2));
        matched_points=best_inliers(1:2,idx(1:n));
    else
        matched_points=matches(1:2,:);
        if (n<0)
            n=size(matched_points,2);
        end
        best_inliers = matched_points;
    end
    
    %% Plot matches - visual check
    if show_matched_points
        figure('name',sprintf('%d matching points from the two images',n));
        imshow(cat(2, im1, im2));

        hold on;
        x1 = f1(1,matched_points(1,:)) ;
        x2 = f2(1,matched_points(2,:)) + size(im1,2) ;
        y1 = f1(2,matched_points(1,:)) ;
        y2 = f2(2,matched_points(2,:)) ;

        line([x1; x2], [y1; y2], 'LineWidth', 1);
        hold off;
    end

    %% Calculate matrix A

    p = zeros(n,3);
    pi = zeros(n,3);
    
    for i = 1:n
        x1 = f1(1,best_inliers(1,i)) ;
        x2 = f2(1,best_inliers(2,i)) ;
        y1 = f1(2,best_inliers(1,i)) ;
        y2 = f2(2,best_inliers(2,i)) ;
        p(i,:)=[x1 y1 1];
        pi(i,:)=[x2 y2 1];
    end
    
end
