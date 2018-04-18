function [R,t] = ICP2(pc1,pc2,samples,sampling,max_repeats,rms,verbose,R,t)
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
        verbose = 0;
    end
    %TODO k-d
    %TODO GPU
    %TODO weights(and others)
    
    %max number of elements; safeguards against different lengths
    n = min(size(pc1,1),size(pc2,1));
    
    if samples>n
        samples = n;
    end
    
    switch sampling
        case 1
            %uniform subsampling
            pc1 = datasample(pc1,samples,1,'Replace',false);
            pc2 = datasample(pc2,samples,1,'Replace',false);
            n = samples;
        case 2
            %random subsampling;
            %store original point clouds in pc1o
            pc1o = pc1;
            pc2o = pc2;
            n = samples;
        case 3
        otherwise
            %all points, nothing to be done here
    end

    if n>2500
        warning('The use of more than than 2500 points requested. More than 200s/iteration may be necessary');
    end
    
    %check if the input clours are of the same dimensionality
    if size(pc1,2) ~= size(pc2,2)
        error('The two points clouds should have same dimensionality; %d~=%d'...
            ,size(pc1,2),size(pc2,2))
    else
        d = size(pc1,2);
    end
    if nargin<8
        R=eye(3);
        t=[0 0 0];
    end
    RMSold = 0;
    RMS = inf;
    k=0;

    if verbose
        fprintf('Variables initialized\n');
        if verbose == 2
            fileID = fopen('log.csv','w');
            fprintf(fileID,'MSE,RMS,time_sec\n');
        end
    end
    %TODO implement oscillation rejection
    while abs(RMSold-RMS)>rms && k<max_repeats
        tic 
        
        if sampling==2
            pc1 = datasample(pc1o,samples,1,'Replace',false);
            pc2 = datasample(pc2o,samples,1,'Replace',false);
        end
        
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
        MSE = diffSum/n;
        RMS = sqrt(MSE);
        
        if verbose
            t = toc;
            fprintf('iteration %d took %f seconds; RMS =%f; MSE=%f\n',k,t,RMS,MSE);
            if verbose==2
                fprintf(fileID,'%f,%f,%f\n',MSE,RMS,t);
            end
        end
    end
    
    %Add new line in console and close file
    if verbose
        fprintf('\n');
        if verbose==2
            fprintf(fileID,'\n');
            fclose(fileID);
        end
    end
    
end
