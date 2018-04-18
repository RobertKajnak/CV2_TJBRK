type = 0;

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
    case 2 
        pc1 = readPcd('Data/data/0000000000.pcd');
        pc2 = readPcd('Data/data/0000000003.pcd');
        %omit points further away than 2m
        pc1 = pc1(pc1(:,3)<2,:);
        pc2 = pc2(pc2(:,3)<2,:);
        %dismiss 4th dimension
        pc1 = pc1(:,1:3);
        pc2 = pc2(:,1:3);
end
%[R,t]  = ICP2(pc1,pc2,samples,sampling,max_repeats,rms)
[R,t] = ICP2(pc1,pc2,500,1,3,0.000005,2);

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
h = zeros(3, 1);
hold on;
plot3(NaN,NaN,NaN,'or');
plot3(NaN,NaN,NaN,'ob');
plot3(NaN,NaN,NaN,'og');

%plot points
fscatter3(pcnew(:,1),pcnew(:,2),pcnew(:,3),red);
fscatter3(pc1(:,1),pc1(:,2),pc1(:,3),blue);
fscatter3(pc2(:,1),pc2(:,2),pc2(:,3),green);

legend({'transformed','target','original'});



