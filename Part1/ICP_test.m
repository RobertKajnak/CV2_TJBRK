%to set parallel pool with custom settings
% pp = parpool(8)
% addpath('ataiya-kdtree-req');
% run('ataiya-kdtree-req/kdtree_compile.m')

% Demo Types:
% 0 - sine
% 1 - sine with noise
% 2 - real dataset
% 3 - real dataset with recorded normals, sampling overwriteen
% 4 - real dataset with calculated normals, sampling overwriteen
type = 3;
gpu=0;

samples = 10000;
iter = 20;
rms = 0.00001;
verb = 1;
%this is overwriten in type==3
sampling = 'uni';
isPlot = 1;

switch type
    case 0
        A1=load('data/source.mat');
        pc1=A1.source';
        A2=load('data/target.mat');
        pc2=A2.target';
    case 1
        % sinusoid+noise
        A1=load('data/source.mat');
        pc1=awgn(A1.source',20,'measured');
        A2=load('data/target.mat');
        pc2=awgn(A2.target',20,'measured');
    otherwise
        pc1 = readPcd('Data/data/0000000001.pcd');
        pc2 = readPcd('Data/data/0000000003.pcd');
        %omit points further away than 2m
        pc1 = pc1(pc1(:,3)<2,:);
        pc2 = pc2(pc2(:,3)<2,:);
        %dismiss 4th dimension
        pc1 = pc1(:,1:3);
        pc2 = pc2(:,1:3);
end
if type==3 
        sampling = 'inf';
        nc1 = readPcd('Data/data/0000000001_normal.pcd');
        nc2 = readPcd('Data/data/0000000005_normal.pcd');
        nc1 = nc1(pc1(:,3)<2,:);
        nc2 = nc2(pc2(:,3)<2,:);
        %dismiss 4th dimension
        nc1 = nc1(:,1:3);
        nc2 = nc2(:,1:3);
        
        [R,t] = ICP2(pc1,pc2,'samples',samples,'sampling',sampling,'max_iter',iter, ...
                'rms',rms,'verbose',verb,'method','knn','nc1',nc1,'nc2',nc2,'plot',isPlot);
end 
if type == 4 
        sampling = 'inf';
        [R,t] = ICP2(pc1,pc2,'samples',samples,'sampling',sampling,'max_iter',iter, ...
                'rms',rms,'verbose',verb,'method','knn','plot',isPlot);
end
%TODO implement GPU parallelization
if gpu
    pc1 = gpuArray(pc1);
    pc2 = gpuArray(pc2);
end

if type<3 
    [R,t] = ICP2(pc1,pc2,'samples',samples,'sampling',sampling,'max_iter',iter, ...
                    'rms',rms,'verbose',verb,'method','knn','plot',isPlot);
end
return
%blue
blue = zeros(size(pc1,1),1);
blue(:,:) = 0.6;
blue(1,1)=1;
blue(2,1)=0;
%green
green = zeros(size(pc2,1),1);
green(:,:) = 0.3;
green(1,1)=1;
green(2,1)=0;
%red
red = zeros(size(pc1,1),1);
red(1,1)=1;

pcnew = pc1;
for i=1:size(pc1)
    pcnew(i,:) = (R*pc1(i,1:3)' + t )';
end

% blue: original
% red: transformed R*original + t
% green: target
figure
%Create legend
hold on;
plot3(NaN,NaN,NaN,'or');
plot3(NaN,NaN,NaN,'ob');

%plot points
fscatter3(pcnew(:,1),pcnew(:,2),pcnew(:,3),red);
%fscatter3(pc1(:,1),pc1(:,2),pc1(:,3),blue);
fscatter3(pc2(:,1),pc2(:,2),pc2(:,3),green);

%legend({'transformed','target','original'});
legend({'transformed','target'});

return
legend({'all points','uniform','random','provided normals','calculated normals'})
xlabel('Iterations');
ylabel('RMS');
hline = findobj(gcf, 'type', 'line')
set(hline(1),'Marker','o')
set(hline(2),'Marker','*')
set(hline(3),'Marker','diamond')
set(hline(4),'Marker','square')
set(hline(5),'Marker','+')