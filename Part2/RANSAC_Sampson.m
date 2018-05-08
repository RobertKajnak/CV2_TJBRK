function [p1_best,p2_best,F,ind]= RANSAC_Sampson(p1, p2, N, iter, threshhold)
% [p1_best,p2_best,F,ind]= RANSAC_Sampson(p1, p2, N, iter)
% 	    Calculate the ransac transformation from the given points and
% fundamental matrix
% Parameters:
%       p1, p2      - points used in p2'*F*p1
%       N           - number of points to keep
%       iter        - number of iterations
% Returns:
%       p1_best     - best base coordinates
%       p2_best     - best target coordinates
%       F           - best fundamental matrix
%       ind         - indices of p1_best and p2_best
% See also KEYPOINT_MATCHING
    if nargin<5
        threshhold = 0.0005;
    end

    best_num_inliers = -inf;
    p1_best=[];
    p2_best=[];
    for n = 1:iter
        [p1m,ind] = datasample(p1, N, 1, 'Replace', false);
        p2m = p2(ind,:);

        % Transform, plot & count inliers
        A = MakeA(p1m,p2m);
        F = MakeF(A);
        
        d = Sampson(F,p1,p2);
        p1_inliers = p1(d<threshhold,:);
        p2_inliers = p2(d<threshhold,:);
        
        % Save the best transformation parameters
        if size(p1_inliers,1) > best_num_inliers
            best_num_inliers = size(p1_inliers,1);
            p1_best = p1_inliers;
            p2_best = p2_inliers;
            best_F=F;
            best_A=A;
        end
    end
    F = best_F;
    %A = best_A;
end
