function [R,t] = ICP2(pc1,pc2,varargin)
% IPC2  Calculate the rotation and translation matrices using IPC
%   PC1 origin point cloud of form [n,d], n=number of points, d=dimension
%   PC2 target point cloud of form [m,d]. if m~=n min(n,m) will be considered 
%   SAMPLES amount of samples used to calculate matrices
%   SAMPLING    0 = all points; SAMPLES will be ignored. If sizes do not
%                   match, the first n=min(samples_pc1,samples_pc2) will be
%                   selected
%               1 = uniform sub-sampling
%               2 = random sub-sampling
%               3 = sub-sampling from informative regions
%
%   See also MERGE
%
%   K-D tree implementation by Pramod Vemulapalli, downloaded from 
%   https://nl.mathworks.com/matlabcentral/fileexchange/26649-kdtree-implementation-in-matlab
    
    %TODO - update doc. Also, since now the number of paramters is not an
    %issue, maybe add paralellization as one ('none','CPU','GPU')
    %% resolve default argumnets and check inputs
    p = inputParser;
    validScalarPosNum = @(x) isnumeric(x) && isscalar(x) && (x > 0);
    %TODO these do noth throw errors when they are supposed to
    possibleMethods = {'asis','bruteforce','knn'};
    validSampling = {'all','uniform','random','informative'};
    p.addParameter('samples',2000,validScalarPosNum);
    p.addParameter('sampling','uniform',@(x)any(validatestring(x,validSampling)));
    p.addParameter('max_iter',20,validScalarPosNum);
    p.addParameter('rms',1e-4,validScalarPosNum);
    p.addParameter('verbose',1,@(x)any(x==[0,1,2]));
    p.addParameter('method','knn',@(x)any(validatestring(x,possibleMethods)));
    p.addParameter('R',eye(3));
    p.addParameter('t',zeros(3,1));
    
    p.parse(varargin{:});
    r = p.Results;
    samples = r.samples;
    sampling = r.sampling;
    max_iter = r.max_iter;
    rms = r.rms;
    verbose =r.verbose;
    method = r.method;
    R= r.R;
    t= r.t;
    
    %% Initialize variables
    if strcmp(method,'knn')
        %pc1 = kd_buildtree(pc1);
        tree = kd_buildtree(pc2,0);
    end
    %TODO GPU
    %TODO weights(and others)
    
    %max number of elements; safeguards against different lengths
    n = min(size(pc1,1),size(pc2,1));
    
    if samples>n
        samples = n;
    end
    
    switch sampling
        case 'uniform'
            %uniform subsampling
            pc1 = datasample(pc1,samples,1,'Replace',false);
            pc2 = datasample(pc2,samples,1,'Replace',false);
            n = samples;
        case 'random'
            %random subsampling;
            %store original point clouds in pc1o
            pc1o = pc1;
            pc2o = pc2;
            n = samples;
        case 'informative'
        otherwise
            %all points, select subset of points that allow pc1 and pc2 
            %sizes to match
            n = min(size(pc1,1),size(pc2,1));
            pc1 = pc1(1:n,:);
            pc2 = pc2(1:n,:);
    end
    if sampling ~= 2
        %Because of the inner for-loop, each worker would create it's
        %own copy of pc2, which increases overhead
        %pc2p = parallel.pool.Constant(pc2);
        %pc2pt = parallel.pool.Constant(pc2');
    end
    
    if n>20000
        warning('The use of more than than 20000 points requested. More than 200s/iteration may be necessary');
    end
    
    %check if the input clours are of the same dimensionality
    if size(pc1,2) ~= size(pc2,2)
        error('The two points clouds should have same dimensionality; %d~=%d'...
            ,size(pc1,2),size(pc2,2))
    else
        d = size(pc1,2);
    end
    
    RMSold = inf;
    diffSum = 0;
    for i=1:n
        diffSum =diffSum + norm( pc1(i,:)' - pc2(i,:)'  )^2;
    end
    MSE = diffSum/n;
    RMS = sqrt(MSE);   
    k=0;

    if verbose
        fprintf('Variables initialized\n');
        fprintf('Baseline RMS=%f; MSE=%f\n',RMS,MSE);
        if verbose == 2
            fileID = fopen('log.csv','w');
            fprintf(fileID,'MSE,RMS,time_sec\n');
            fprintf(fileID,'%f,%f,0.0\n',MSE,RMS);
        end
    end
    %TODO implement oscillation rejection
    %% Main forloop
    P = pc1;
    while abs(RMSold-RMS)>rms && k<max_iter
        k=k+1;
        tic 
        
        if sampling==2
            pc1 = datasample(pc1o,samples,1,'Replace',false);
            pc2 = datasample(pc2o,samples,1,'Replace',false);
            %pc2p = parallel.pool.Constant(pc2);
        end
        
        RMSold = RMS;
        
        Q = zeros([n,d]);
        if strcmp(method,'asis')
            Q = pc2(1:n,:);
        end
        
        %find best matches from A2 for transfomation from A1
        if strcmp(method,'bruteforce')
            parfor i=1:n
                e=inf;
                ind=0;
                for j=1:n
                    diff = norm(pc2(j,:)'-P(i,:)');
                    if (diff<e)
                        e = diff;
                        ind = j;
                    end
                end
                Q(i,:) = pc2(ind,:);
            end
        elseif strcmp(method,'knn')
            parfor i=1:n
                [~,vec_vals,~] = kd_closestpointgood(tree,P(i,:));
                Q(i,:) = vec_vals;
            end
        end
                
        
        p_bar = mean(pc1);
        q_bar = mean(Q);
        
        X = pc1 - p_bar;
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
            P(i,:) = (R*pc1(i,:)' + t)';
            diffSum =diffSum + norm( P(i,:)' - Q(i,:)'  )^2;
        end
        MSE = diffSum/n;
        RMS = sqrt(MSE);
        
        if verbose
            time = toc;
            fprintf('iteration %d took %f seconds; RMS =%f; MSE=%f\n',k,time,RMS,MSE);
            if verbose==2
                fprintf(fileID,'%f,%f,%f\n',MSE,RMS,time);
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

