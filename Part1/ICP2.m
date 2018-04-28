function [R,t] = ICP2(pc1,pc2,varargin)
% IPC2  Calculate the rotation and translation matrices using IPC
%   PC1 origin point cloud of form [n,d], n=number of points, d=dimension
%   PC2 target point cloud of form [m,d]. if m~=n min(n,m) will be considered 
%   VARARGIN -- Following arguments are optional:
%   'nc1','nc2'- array of normals. Should be same size as pointcloud, with each
%                index corresponding to the same indexed point in pc1 and
%                pc2
%   'samples'  - amount of samples used to calculate matrices. Default = 2000
%   'sampling' - 'all' = all points; SAMPLES will be ignored. If sizes do not
%                       match, the first n=min(samples_pc1,samples_pc2) will be
%                       selected
%               'uniform' = uniform sub-sampling. (DEFAULT)
%               'random' = random sub-sampling: subsampling done at each
%                       iteration
%               'informative' = sub-sampling from informative regions
%   'rms'      - the algorithm stops if rms difference between 2 iterations
%                is lower than this paramter. Default = 30
%   'max_iter' - maximum number of iterations, if rms target 
%                not reached 
%   'method'   - 'bruteforce' => O(n^2) iterative search for minimum
%              - 'knn' => DEFAULT. O(n*log(n)) k-d tree based search. 
%                  Compiled library necessary
%              - 'asis' => The input point clouds are ordered and matching
%                 point search is not necessary
%   'verbose'  - 0=> no output.
%              - 1=> Console output with time, RMS, MSE each iteration
%              - 2=> 1 + saves the values to 'log.csv'
%   'R' & 't'- To continue where a previous search left off or from
%                custom initial transformation. 
%                Default: R=eye(3); t=zeros(1,3)
%
%   See also MERGE
%
%   K-D tree implementation by Andrea Tagliasacchi, downloaded from 
%   https://nl.mathworks.com/matlabcentral/fileexchange/21512-ataiya-kdtree
    
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
    p.addParameter('max_iter',30,validScalarPosNum);
    p.addParameter('rms',1e-5,validScalarPosNum);
    p.addParameter('verbose',1,@(x)any(x==[0,1,2]));
    p.addParameter('method','knn',@(x)any(validatestring(x,possibleMethods)));
    p.addParameter('R',eye(3));
    p.addParameter('t',zeros(3,1));
    p.addParameter('nc1',nan);
    p.addParameter('nc2',nan); 
    
    p.parse(varargin{:});
    r = p.Results;
    samples = r.samples;
    sampling = validatestring(r.sampling,validSampling);
    max_iter = r.max_iter;
    rms = r.rms;
    verbose =r.verbose;
    method = validatestring(r.method,possibleMethods);
    R= r.R;
    t= r.t;
    nc1=r.nc1;
    nc2=r.nc2;
    isNormalsRequested = strcmp('informative',sampling);
    if strcmp(sampling,'all')
        samples = min(size(pc1,1),size(pc2,1));
    end
    if verbose
        onoff=["off","on"];
        fprintf(['Starting search with hyperparameters\nsamples=%d; sampling=%s; '...
            'maximum iterations=%d; rms=%f; method=%s; logging=%s\n'],...
            samples,sampling,max_iter,rms,method,onoff(int8(verbose==2)+1));
    end
    %% Initialize variables
    %TODO GPU
    %TODO weights(and others)
    
    %% Point cloud validation
    %max number of elements; safeguards against different lengths
    n = min(size(pc1,1),size(pc2,1));
        %check if the input clours are of the same dimensionality
    if size(pc1,2) ~= size(pc2,2)
        error('The two points clouds should have same dimensionality; %d~=%d'...
            ,size(pc1,2),size(pc2,2))
    else
        d = size(pc1,2);
    end
    
    if samples>n
        samples = n;
    end
    n = samples;
    
    %Check for NaNs
    if (any(any(isnan(nc1)))~=all(all(isnan(nc1)))) ||  ...
        (any(any(isnan(nc2)))~=all(all(isnan(nc2))))
        warning('NaN values in normal clouds not might give less-than optimal results');
        nc1(isnan(nc1))=0;
        nc2(isnan(nc2))=0;
    end
    %Check if sizes match
    isNormalsProvided = all(all(~isnan(nc1))) && all(all(~isnan(nc2)));
    if isNormalsProvided
        if (size(pc1,1)~= size(nc1,1))
            error(['The number of points in the pointcloud 1 do not match the'...,
                'number of points in normal cloud: %d=/=%d'],size(pc1,1),size(nc1,1));
        end
        if (size(pc2,1) ~= size(nc2,1))
            error(['The number of points in the pointcloud 2 do not match the'...,
                'number of points in normal cloud: %d=/=%d'],size(pc2,1),size(nc2,1));
        end
    end
    %Since I hate when a matlab library gives an error 5 levels deep inside
    %the file dependenceies, instead of giving at least a warning upfornt,
    %I will try to do better
    if ~isNormalsRequested && isNormalsProvided
        warning(['You have provided normals but have not requested a '...
            'sampling method that makes use of it e.g. "informative"'])
    end
    %% Resize pointclouds
    %n and d verified/computed at this point
    

    switch sampling
        case validSampling(2)
            %uniform subsampling
            [pc1,~] = datasample(pc1,n,1,'Replace',false);
            [pc2,~] = datasample(pc2,n,1,'Replace',false);
            

%             if isNormalsRequested
%                 nc1 = nc1(idx1);
%                 nc2 = nc2(idx2);
%             end
        case validSampling(3)
            %random subsampling;
            %store original point clouds in pc1o
            pc1o = pc1;
            pc2o = pc2;
            pc1 = pc1(1:n,:);
            pc2 = pc2(1:n,:);
            
%             if isNormalsRequested
%                 nc1o = nc1;
%                 nc2o = nc2;
%                 nc1 = nc1(1:n,:);
%                 nc2 = nc2(1:n,:);
%             end
        case validSampling(4)
            %informative
            if isNormalsRequested && ~isNormalsProvided
                %TODO Implement normal calcuation for pointcloud
                nc1=[]
                nc2=[] 
            else
                %remove non-informative 0 values
            end
            pc1o=pc1;
            pc1=cell(1,3);
            pc2o=pc2;
            pc2=cell(1,3);
            for i=1:3
                ind1 = getUsefulIndicesSorted(nc1(:,i),n);
                pc1{i} = pc1o(ind1,:);
                ind2 = getUsefulIndicesSorted(nc2(:,i),n);
                pc2{i} = pc2o(ind2,:);
            end
            pc1 = [pc1{1};pc1{2};pc1{3}];
            pc2 = [pc2{1};pc2{2};pc2{3}];
            n = min(size(pc1,1),size(pc2,1));
            pc1 = pc1(1:n,:);
            pc2 = pc2(1:n,:);
        otherwise
            %'as is'
            %all points, the lower number of points in n has been dealt in
            %the previous section

            pc1 = pc1(1:n,:);
            pc2 = pc2(1:n,:);
%             if isNormalsReqested
%                 nc1 = nc1(1:n,:);
%                 nc2 = nc2(1:n,:);
%             end
    end
    if sampling ~= 2
        %Because of the inner for-loop, each worker would create it's
        %own copy of pc2, which increases overhead
        %pc2p = parallel.pool.Constant(pc2);
        %pc2pt = parallel.pool.Constant(pc2');
    end
    
    if n>20000 && ~strcmp(method,'knn')
        warning('The use of more than than 20000 points requested. More than 200s/iteration may be necessary');
    end
   
    %% Initialize variables for for loop
    RMSold = inf;
    diffSum = 0;
    n
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
    if strcmp(method,'knn')
        tree = KDTree(pc2);
        %tree= kd_buildtree(pc2,0);
    end
    %TODO implement oscillation rejection
    %% Main forloop
    P = pc1;
    while abs(RMSold-RMS)>rms && k<max_iter
        k=k+1;
        tic 
        
        if strcmp(sampling,'random')
            pc1 = datasample(pc1o,n,1,'Replace',false);
            pc2 = datasample(pc2o,n,1,'Replace',false);
            for i=1:n
                P(i,:) = (R*pc1(i,:)' + t)';
            end
            %pc2p = parallel.pool.Constant(pc2);
            if strcmp(method,'knn')
                tree = KDTree(pc2);
            end
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
%             parfor i=1:n
%                 [~,vec_vals,~] = kd_closestpointgood(tree,P(i,:));
%                 Q(i,:) = vec_vals;
%             end
            for i=1:n
                ind = tree.nn(P(i,:));
                
                Q(i,:) = pc2(ind,:);
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

function [ind] = getUsefulIndicesSorted(val,nrpoints)
    [val,ind] = sort(val);
    from = find(val==0,1,'first');
    to = find(val==0,1,'last');
    ind = [ind(1:from); ind(to:end)];
    len = size(ind,1);
    rat = ceil(3*len/nrpoints);
    ind = ind(1:rat:len);
end