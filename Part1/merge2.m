function res =  merge2()
    skip = 1;
    sampling = 'uniform';
    samples = 2000;
    iterations = 30;
    rms = 0.0001;
    verbose = 1;
    method = 'knn';
    res = merge_all31(skip, sampling, samples, iterations,rms,verbose,method);
end

function gridsearch()
    
for sk = 1:6
   sp = [1,2,4,6,8,10];
   skip = sp(sk);
   sampling = 0;
   for s = 1:5
       sa = [500,1000,2000,5000,10000];
       samples = sa(s);
       for iterations = 10:10:30
           try
            res = merge_all31(skip, sampling, samples, iterations,0.0001,0);
            save(sprintf('31__%d_%d_%d.mat',skip,samples,iterations),'res');
           catch ME
                   getReport(ME)
                'exception'
                skip
                samples
                iterations
           end
           try
            res = merge_all32(skip, sampling, samples, iterations,0.0001,0);
            save(sprintf('32__%d_%d_%d.mat',skip,samples,iterations),'res');
           catch ME
                    getReport(ME)
                'exception'
                skip
                samples
                iterations
           end
           try

            res = merge_all32_improved(skip, sampling, samples, iterations,0.01,0.0001,0);
            save(sprintf('32imp__%d_%d_%d.mat',skip,samples,iterations),'res');
           catch ME
                 getReport(ME)
                'exception'
                skip
                samples
                iterations
           end

       end
   end
end
end


function pc = get_pointcloud(number)
    pc = readPcd(sprintf('Data/data/%010d.pcd', number));
    %dismiss 4th dimension
    pc = pc(:,1:3);
    %omit points further away than 2m
    pc = pc(pc(:,3)<2,:);
end


function res = merge_all31(n, sampling, samples, iterations,rms,verbose,method)
    merged = get_pointcloud(0);
%     EE = zeros(floor(100/n), iterations + 1);
    for i = n:n:99
        base = get_pointcloud(i-n);
        target = get_pointcloud(i);
        [R,t] = ICP2(base,target);
%         [R,t] = ICP2(base,target,'sampling',sampling,'samples',samples,'max_iter',...
%             iterations,'rms',rms,'verbose',verbose,'method',method);
%        EE( i/n,:) = E';
        for j=1:size(merged,1)
            merged(j,:) = (R*merged(j,1:3)' + t )';
        end
        merged = [merged ; target];
    end
    res = merged;
end

function res = merge_all32(n, sampling, samples, iterations,rms,verbose)
    p = get_pointcloud(0);
%     EE = zeros(floor(100/n), iterations + 1);
    
    for i = n:n:99
        q = get_pointcloud(i);
        [R,t] = ICP2(base,target,samples,sampling,iterations,rms,verbose);
%        EE( i/n,:) = E';
        for j=1:size(p,1)
            p(j,:) = (R*p(j,1:3)' + t )';
        end
        p = [p ; q];
    end
    res = p;
end

% remove points too close to another
function res = merge_all32_improved(n, sampling, samples, iterations,distance,rms, verbose)
%     EE = zeros(floor(100/n), iterations + 1);

    p = get_pointcloud(0);

    for i = n:n:99
        q = get_pointcloud(i);
        [R,t] = ICP2(base,target,samples,sampling,iterations,rms,verbose);
%        EE( i/n,:) = E';
        for j=1:size(p,1)
            p(j,:) = (R*p(j,1:3)' + t )';
        end
        p = [p ; q];
    
    j = size(p,1);
    while(j>1) 
        
        pp = p(j,:);
        %size(res,1)
        p = [ p( sqrt(  sum(   (p-pp).^2,2)   )>distance ,:)   ;pp];
        %size(x,1)
        
        if size(p,1)<j ; j = size(p,1); else; j=j-1; end
    end
    
    
    end  
        

    
    res = p;
end



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
% fscatter3(res(:,1),res(:,2),res(:,3),res(:,3));
% fscatter3(pc1(:,1),pc1(:,2),pc1(:,3),blue);
% fscatter3(pc2(:,1),pc2(:,2),pc2(:,3),green);
end