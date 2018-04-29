%call merge2() to gridsearch approaches 3.1 and 3.2 with specified params
function merge2()

%choose desired parameters to run ICP on
%bruteforce or knn
method = 'knn';
verbose = 1;
%cutoff RMS delta between 2 iterations
rms = 0.0001;
%skip in-between frames
sp = [1,2,4,6,8,10];
%number of samples
sa = [500,1000,2000,5000,10000,20000];
%number of max ICP iterations unless rms is unchanged
it = [10,20,30];
%choose sampling method
sampling = 'uniform';
%only used for comp. intense lower density 3.2 improvement
%uncomment code below and set distance to desired radius
distance = 0.01;

%loops over all parameter settings
for sk = 1:size(sp,2);
   skip = sp(sk);
   for s = 1:size(sa,2);
       samples = sa(s);
       for i = 1:size(it,2);
           iterations = it(i);
           
           % build model according to 3.1. approach 
           try
            res = merge_all31(skip, sampling, samples, iterations,rms,verbose,method);
            save(sprintf([sampling,'_',method,'_','31_%d_%d_%d.mat'],skip,samples,iterations),'res');
           catch ME
                   getReport(ME)
                'exception'
                skip
                samples
                iterations
           end
           % build model according to 3.2. approach
           try
            res = merge_all32(skip, sampling, samples, iterations,rms,verbose,method);
            save(sprintf([sampling,'_',method,'_','32_%d_%d_%d.mat'],skip,samples,iterations),'res');
           catch ME
                    getReport(ME)
                'exception'
                skip
                samples
                iterations
           end
           %uncomment below for 3.2 improvement, takes a very long time
           
%            try
% 
%             res = merge_all32_improved(n, sampling, samples,...
%             iterations,distance,rms, verbose);
%             save(sprintf([sampling,'_',method,'_','imp_%d_%d_%d.mat'],skip,samples,iterations),'res');
%            catch ME
%                  getReport(ME)
%                 'exception'
%                 skip
%                 samples
%                 iterations
%            end

       end
   end
end
end

%load pointcloud files
function pc = get_pointcloud(number)
    pc = readPcd(sprintf('Data/data/%010d.pcd', number));
    pc = pc(:,1:3);
    %omit points further away than 2m
    pc = pc(pc(:,3)<2,:);
end

% as specified in 3.1, compute transformation from consecutive frames
function res = merge_all31(n, sampling, samples, iterations,rms,verbose,method)
    merged = get_pointcloud(0);
    for i = n:n:99
        i
        base = get_pointcloud(i-n);
        target = get_pointcloud(i);
        [R,t] = ICP2(base,target,'sampling',sampling,'samples',samples,'max_iter',...
            iterations,'rms',rms,'verbose',verbose,'method',method);
        for j=1:size(merged,1)
            merged(j,:) = (R*merged(j,1:3)' + t )';
        end
        merged = [merged ; target];
    end
    res = merged;
end
% as specified in 3.2, compute transformation from whole merged model and
% next frame
function res = merge_all32(n, sampling, samples, iterations,rms,verbose,method)
    p = get_pointcloud(0);

    for i = n:n:99
        i
        q = get_pointcloud(i);
        [R,t] = ICP2(p,q,'sampling',sampling,'samples',samples,'max_iter',...
            iterations,'rms',rms,'verbose',verbose,'method',method);

        for j=1:size(p,1)
            p(j,:) = (R*p(j,1:3)' + t )';
        end
        p = [p ; q];
    end
    res = p;
end

% remove points too close to another
function res = merge_all32_improved(n, sampling, samples, iterations,distance,rms, verbose)

    p = get_pointcloud(0);

    for i = n:n:99
        i
        q = get_pointcloud(i);
        [R,t] = ICP2(p,q,'sampling',sampling,'samples',samples,'max_iter',...
            iterations,'rms',rms,'verbose',verbose,'method',method);
        for j=1:size(p,1)
            p(j,:) = (R*p(j,1:3)' + t )';
        end
        p = [p ; q];
    
    j = size(p,1);
    while(j>1) 
        
        pp = p(j,:);
        p = [ p( sqrt(  sum(   (p-pp).^2,2)   )>distance ,:)   ;pp];

        if size(p,1)<j ; j = size(p,1); else; j=j-1; end
    end
    
    
    end  
    res = p;
end


% plot model with colour of data points as depth
function plot_res(res)
%blue
blue = zeros(size(res,1),1);
blue(:,:) = 0.6;
blue(1,1)=1;
blue(2,1)=0;
%green
green = zeros(size(res,1),1);
green(:,:) = 0.3;
green(1,1)=1;
green(2,1)=0;
%red
red = zeros(size(res,1),1);
red(1,1)=1;

fscatter3(res(:,1),res(:,2),res(:,3),res(:,3));
end