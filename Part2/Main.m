%% Add path variables and clear workspace
addpath(genpath('./Assignment_4'))
%clear all; close all;
%% Preparing file names
path = 'data/House/';

files = dir(path);
%The initial files is taken twice (1->2 and end->1)
M = length(files)-1;
images = cell(1,M);
for i=3:length(files)
    images{i-2} = files(i).name;
end
images{end} = files(3).name;

%TODO - eliminate points from background -    %active-contour or sift param
%% Load through all files TODO this comment makes little sense
isFirstIter = true;
%TODO - do dynamic reallocation within loop
PVM=zeros((M-1)*2,1000);
showEpipolar = false;
for i=1:M-1
    fprintf('Preparing matches between %d<->%d\n',i,i+1);
    %% Load image pairs
    im1name = sprintf([path,images{i}],i);
    im2name = sprintf([path,images{i+1}],i+1);
    %TODO do RGBcheck
    im1 = imread(im1name);
    im2 = imread(im2name);

    %% Get points of interest
    %Specify number of sample points;
    n = 50;

    %TODO change filtering approach in InterestPoints to be parameter:
    %currently: 3 - most outliers removed
    %default(1.5) => crappy
    %try background removeal instead
    if isFirstIter%TODO - change this to a flag
        [f1,d1] = vl_sift(single(im1));
    else
        f1=f2;
        d1=d2;
    end
    
    [f2,d2] = vl_sift(single(im2));
    matches = vl_ubcmatch(d1, d2,3);
    
    [p_base, p_target] = InterestPoints(f1,f2,matches, -1,0,im1,im2);

    %%  3.1 Eight Point Algorithm
    A = MakeA(p_base,p_target);

    F = MakeF(A);

    %% 3.2.1 Normalize source points

    [p_base_hat,T] = normalizedPi(p_base);
    [p_target_hat,T_prime] = normalizedPi(p_target);

    %% 3.2.2 Make A Matrix

    A_hat = MakeA(p_base_hat,p_target_hat);

    F_hat = MakeF(A_hat);

    F_prime = T_prime'*F_hat*T;
    
    %p_base_prime = (T_prime^-1*p_base_hat')';

    %% 3.3 RANSAC and Normalize
   
    [p_base_rans,p_target_rans,F_rans] = RANSAC_Sampson(p_base,p_target,8,50);
    
    %also normalize them
    [p_base_rans_hat,T_rans] = normalizedPi(p_base_rans);
    [p_target_hat_rans,T_prime_rans] = normalizedPi(p_target_rans);
    
    A_hat_rans = MakeA(p_base_rans_hat,p_target_hat_rans);

    F_hat_rans = MakeF(A_hat_rans);

    F_prime_rans = T_prime_rans'*F_hat_rans*T_rans;
    
    %p_base_rans_prime = (T_prime_rans^-1*p_base_rans_hat')';
    %% 3.end Calculate the epipolar lines and draw them    
    if showEpipolar
        %simple eight-point
        drawEpipolar(F,p_base(1:8,:),im1,'Epipolar lines using simple eight-point algorithm');

        %normalized eight-point
        drawEpipolar(F_prime,p_base(1:8,:),im1,'Epipolar lines using normalized eight-point algorithm');

        %normalized RANSACed eight-point
        drawEpipolar(F_prime_rans,p_base_rans(1:8,:),im1,['Epipolar lines using eight-point algoirthm augmented by'...
                'normalization and RANSAC point selection']);
        %TODO - looks really unstable. Why? (even for the ransaced one)
        %it should be the same, as it should dependent on the points selected,
        %if they are selected correctly and the F should be basically the same
        %for any runs, given the same two images. On the other hand, the points
        %do line up with the lines (pun not intended) better on better
        %algorithms, which suggests that the algorithm is correct
    end
    %% 4. 
    if isFirstIter
        for j=1:size(p_base_rans,1)
            PVM(i*2-1,j) = p_base_rans(j,1);
            PVM(i*2,j) = p_base_rans(j,2);
        end
        PVMind = j;
    end
    
    for j=1:size(p_target_rans,1)
        %TODO change this
        matchesPrevPoint=true;
        prevPoint = j;
        if matchesPrevPoint
            PVM(i*2+1,prevPoint) = p_target_rans(j,1);
            PVM(i*2+2,prevPoint) = p_target_rans(j,2);
        else
            PVM(i*2+1,PVMind) = p_target_rans(j,1);
            PVM(i*2+2,PVMind) = p_target_rans(j,2);
            PVMind=PVMind+1;
        end
    end
    %return
    
    isFirstIter = false;
end

%% plot PVM
figure('name','Point-view matrix representation');
imshow(PVM<1)
return
%% read matchview.txt
f=fopen('PointViewMatrix.txt','r');
PVM = fscanf(f,'%f');
fclose(f);

PVM = reshape(PVM,[215,202]);
PVM = PVM';

%% testcase
figure;
imshow(imread(sprintf([path,images{1}],1)));
hold on;
for i=1:215
    color = rand(1,3);

    for j = 1:2:101
        color=color*.98;
        if PVM(j,i)~=0 && PVM(j+1,i) ~=0
            plot(PVM(j,i),PVM(j+1,i),'x','color',color)
        end
    end
end




