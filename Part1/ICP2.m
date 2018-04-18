function [R,t] = ICP2(pc1,pc2,samples,sampling,max_repeats,rms,verbose)
% IPC2  Calculate the rotation and translation matrices using IPC
%   PC1 origin point cloud of form [n,d], n=number of points, d=dimension
%   PC2 target point cloud of form [m,d]. if m~=n min(n,m) will be considered 
%   SAMPLES amount of samples used to calculate matrices
%   SAMPLING    0 = all points; SAMPLES will be ignored
%               1 = uniform sub-sampling
%               2 = random sub-sampling
%               3 = sub-sampling from informative regions
%
%   See also MERGE    

    if nargin < 7
        verbose =0;
    end
    
%TODO different kinds of sampling, uniform(?) below?
    %max number of elements; safeguards against different lengths
    n = min(size(pc1,1),size(pc2,1));
    
    if samples>n
        samples = n;
    end
    
    switch sampling
        case 1
            ran = randsample(1:n,samples);
            pc1 = pc1(ran,:);
            ran = randsample(1:n,samples);
            pc2 = pc2(ran,:);
            n = samples;
        case 2
        case 3
        otherwise
            %all points, nothing to be done here
    end

    %if the sizes
    %check if the input clours are of the same dimensionality
    if size(pc1,2) ~= size(pc2,2)
        error('The two points clouds should have same dimensionality; %d~=%d'...
            ,size(pc1,2),size(pc2,2))
    else
        d = size(pc1,2);
    end
    
    %TODO add the weights W
    
    R=eye(3);
    t=[0 0 0];
    RMSold = 0;
    RMS = inf;
    k=0;
    

    if verbose
        fprintf('Variables initialized\n');
    end
    while abs(RMSold-RMS)>rms && k<max_repeats
        tic 
        
        k=k+1;
        RMSold = RMS;
        
        P = pc1;
        Q = zeros([n,d]);
       
        %find best matches from A2 for transfomation from A1
        ind = 0;
        for i=1:n
            e = inf;
            for j=1:n
                diff = norm( (R*pc1(i,:)' + t') - pc2(j,:)');
                if (diff<e) 
                    e = diff;
                    ind = j;
                end
            end
            Q(i,:) = pc2(ind,:);
        end 
        
        p_bar = mean(P);
        q_bar = mean(Q);

        X = P - p_bar;
        Y = Q - q_bar;

        %Step 3
        %multiplication should be the correct way round
        S = X'*Y;

        %Step 4;
        [U,~,V]=svd(S);
        Matrix = eye(d);
        Matrix(d,d) = det(V*U');
        R = V*Matrix*U';

        t = q_bar' - R*p_bar';

        %calculate RMS
        diffSum = 0;
        for i=1:n
            P(i,:) = (R*P(i,:)' + t)';
            diffSum =diffSum + norm( P(i,:)' - Q(i,:)'  )^2;
        end
        RMS=sqrt(diffSum/n);
        
        if verbose
            fprintf('RMS= %f\n',RMS);
            fprintf('iteration %d took %f seconds\n',k,toc);
        end
    end
    %fprintf(' \n');
    
    
end