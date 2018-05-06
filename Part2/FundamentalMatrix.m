function [A] = FundamentalMatrix(im1, im2)
% Calculates the fundamental matrix A for the transformation between images
% im1 and im2


%% Get matches
[scores, matches,f1,f2,~,~] = keypoint_matching(im1, im2);


%% Calculate Ransac
iterations = 10;
samples = 20; 
[best_transform, best_inliers] = RANSAC(im1,im2,matches, f1, f2, iterations, samples);
[~,idx] = sort(best_inliers(3,:));
sorted_matches=best_inliers(1:2,idx);
matched_points = datasample(sorted_matches,50, 2, 'Replace', false);

%% Plot matches - visual check

figure('name','10 matching points from the two images');
imshow(cat(2, im1, im2));

hold on;
x1 = f1(1,matched_points(1,:)) ;
x2 = f2(1,matched_points(2,:)) + size(im1,2) ;
y1 = f1(2,matched_points(1,:)) ;
y2 = f2(2,matched_points(2,:)) ;

line([x1; x2], [y1; y2], 'LineWidth', 1);

%% Calculate matrix A
hold off;

n = size(matched_points,2);
A = zeros(n,9);
for i = 1:n
    x1 = f1(1,best_inliers(1,i)) ;
    x2 = f2(1,best_inliers(2,i)) ;
    y1 = f1(2,best_inliers(1,i)) ;
    y2 = f2(2,best_inliers(2,i)) ;
    A(i,:) = [x1*x2, x1*y2 x1 y1*x2 y1*y2 y1 x2 y2 1];
end

