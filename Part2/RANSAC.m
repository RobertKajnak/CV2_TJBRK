function [best_transformation, best_inliers]= RANSAC(image1,image2,matches, f1, f2, N, P, show_image,show_all_matching_points)
% RANSAC - calculate the ransac transformation from the given features
%   F1, F2 - features returned from KEYPOINT_MATCHING
%   N - Number of iterations
%   F - number of point pairs considered
%   SHOW_IMAGE - 0 no image;1 own image; 2 own and matlab image
%   SHOW_ALL_MATCHING_POINTS
% See also KEYPOINT_MATCHING
    if nargin<8
        show_image=false;
    end
    if nargin<9
        show_all_matching_points = false;
    end
    if P>size(matches,2)
        warning('Number of samples larger than dataset. Using all points');
        P=size(matches,2);
    end

    best_num_inliers = -inf;
    best_inliers = [];
    best_m = [];
    best_t = [];
    for n = 1:N
        P_matches = datasample(matches, P, 2, 'Replace', false);
        % Construct matrices A and b
        A = [];
        b = [];
        for i = 1:P
            x1 = f1(1,P_matches(1,i)) ;
            x2 = f2(1,P_matches(2,i)) ;
            y1 = f1(2,P_matches(1,i)) ;
            y2 = f2(2,P_matches(2,i)) ;
            A_i = [x1 y1 0 0 1 0; 0 0 x1 y1 0 1];
            b_i = [x2;y2];
            A = [A; A_i];
            b = [b; b_i];
        end
        % calculate transformation parameters
        x = pinv(A)*b;
        m = [x(1) x(2); x(3) x(4)];
        t = [x(5);x(6)];
        % Transform, plot & count inliers
        inliers = [];
        if show_all_matching_points
            figure();
            imshow(cat(2, image1, image2));
            hold on;
        end
        
        for j = 1:length(matches)
            x1 = f1(1,matches(1,j)) ;
            y1 = f1(2,matches(1,j)) ;
            transformed = m*[x1;y1]+t;
            x_trans = transformed(1);
            y_trans = transformed(2);
            if show_all_matching_points
                line([x1; x_trans + size(image1, 2)], [y1;y_trans], 'Color', 'r', 'LineWidth', 1);
            end
            x2 = f2(1,matches(2,j)) ;
            y2 = f2(2,matches(2,j)) ;
            if abs(x2-x_trans) < 10 && abs(y2-y_trans) < 10
                inliers = [inliers [matches(1,j); matches(2,j); sqrt((x2-x_trans)^2+(y2-y_trans)^2)]];
            end
        end
        if show_all_matching_points
            hold off;
        end
        % Get best transformation parameters
        if length(inliers) > best_num_inliers
            best_num_inliers = length(inliers);
            best_inliers = inliers;
            best_m = m;
            best_t = t;        
        end

         
    end
    
    A = [];
    b = [];
   for i = 1:best_num_inliers
        x1 = f1(1,best_inliers(1,i)) ;
        x2 = f2(1,best_inliers(2,i)) ;
        y1 = f1(2,best_inliers(1,i)) ;
        y2 = f2(2,best_inliers(2,i)) ;
        A_i = [x1 y1 0 0 1 0; 0 0 x1 y1 0 1];
        b_i = [x2;y2];
        A = [A; A_i];
        b = [b; b_i];
    end
    
     x = pinv(A)*b;
     best_m = [x(1) x(2); x(3) x(4)];
     best_t = [x(5);x(6)];
        
    
    fprintf('The image was rotated with %.2f%% precision\n',best_num_inliers*100.0/size(matches,2));
    % transform image and show next to transformation with matlab
    if show_image
        new_image = transform_image(image1, best_m, best_t);
        imshow(new_image);
        if show_image==2
            % matlab recommends: use affine2d instead of maketform and
            % imwarp instead of imtransform
            tform = affine2d([best_m'; best_t']);
            matlab_transformed = imwarp(image1, tform);%, 'OutputView', imref2d(size(image1)));
            figure('name','Transformation done using matlab');
            %matlab_transformed = imresize(matlab_transformed,0.6);
            imshow(matlab_transformed);
            figure('name','Original Image');
            imshow(image2);
        end
    end
    
    best_transformation.m = best_m; 
    best_transformation.t = best_t;
end
