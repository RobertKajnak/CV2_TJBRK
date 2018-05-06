%% This demo runs both keypoint_matching and RANSAC 

% if ~exists(vl_version)
%     run('..\..\vlfeat\toolbox\vl_setup');
% end

%% Load images
boat1 = imread('boat1.pgm');
boat2 = imread('boat2.pgm');
 
%% find matching points
[scores, matches,f1,f2,d1,d2] = keypoint_matching(boat1, boat2);
matches_subset = datasample(matches,50, 2, 'Replace', false);
 
%% Plot matches
figure('name','50 matching points from the two images');
imshow(cat(2, boat1, boat2));

hold on;
x1 = f1(1,matches_subset(1,:)) ;
x2 = f2(1,matches_subset(2,:)) + size(boat1,2) ;
y1 = f1(2,matches_subset(1,:)) ;
y2 = f2(2,matches_subset(2,:)) ;

line([x1; x2], [y1; y2], 'Color', 'r', 'LineWidth', 1);

hold off;
 
%% RANSAC
figure('name','Converting boat1 to match boat2');
[best_transform, best_in] = RANSAC(boat1, boat2, matches, f1, f2, 5, 20, 1);

%% Other way around
figure('name','Converting boat2 to match boat1');
[scores, matches,f1,f2,d1,d2] = keypoint_matching(boat2, boat1);
[best_transform2] = RANSAC(boat2, boat1, matches, f1, f2, 20, 3, 1);
