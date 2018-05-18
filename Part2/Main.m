% Add path variables and clear workspace
run('.\vlfeat\toolbox\vl_setup');
clear all; close all;

%% Debug/partial result flags
showSift = false;
showEpipolar = false;
stopAfterFirstIteration = false;

showPVM = true;
% use example PVM from file, false runs on house images
examplePVM = false;
% number of consecutive images to stitch together for SFM
nr_consec_imgs = 48;

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

%% Initialize some variables
isFirstIter = true;
%the maximum number of features that can appear per iteration -- this also
%determines the size of PVM. If index out of bounds, increase this
maxExpectedFeatures=2000;
PVM=zeros((M-1)*2,maxExpectedFeatures);
matchesf2Last=zeros(1,maxExpectedFeatures);
PVMind = 1;
%% Main for loop to go through all the image pairs
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
    n = 8;

    % 5: actually starts working...
    % currently: 3 - most outliers removed
    % default(1.5) => crappy
    %try background removeal instead
    if isFirstIter
        [f1,d1] = vl_sift(single(im1));
    else
        f1=f2;
        d1=d2;
    end
    
    [f2,d2] = vl_sift(single(im2));
    if ~isFirstIter
        prevMatches = matches(2,:);
    end
    matches = vl_ubcmatch(d1, d2,5);
    if isFirstIter
        prevMatches = matches(1,:);
    end

    [p_base, p_target] = InterestPoints(f1,f2,matches, 30,showSift,im1,im2);
    if size(p_base,1)<8
         warning('Less than 8 matching points found. Skipping iteration')
         continue;
    end
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

    %% 3.3 RANSAC and Normalize
   
    [p_base_rans,p_target_rans,F_rans] = RANSAC_Sampson(p_base,p_target,8,500);
    
    %also normalize them
    [p_base_rans_hat,T_rans] = normalizedPi(p_base_rans);
    [p_target_hat_rans,T_prime_rans] = normalizedPi(p_target_rans);
    
    A_hat_rans = MakeA(p_base_rans_hat,p_target_hat_rans);

    F_hat_rans = MakeF(A_hat_rans);

    F_prime_rans = T_prime_rans'*F_hat_rans*T_rans;
    
    
    %% 3.end Calculate the epipolar lines and draw them    
    if showEpipolar
        %simple eight-point
        drawEpipolar(F,p_base(1:8,:),im1,'Epipolar lines using simple eight-point algorithm');

        %normalized eight-point
        drawEpipolar(F_prime,p_base(1:8,:),im1,'Epipolar lines using normalized eight-point algorithm');

        %normalized RANSACed eight-point
        drawEpipolar(F_prime_rans,p_base_rans(1:8,:),im1,['Epipolar lines using eight-point algoirthm augmented by'...
                'normalization and RANSAC point selection']);
    end

  %% 4.
    p_sel_base = p_base;
    p_sel_target = p_target;
    if isFirstIter
        for j=1:size(p_sel_base,1)
            PVM(1,PVMind) = p_sel_base(j,1);
            PVM(2,PVMind) = p_sel_base(j,2);
            
            PVM(3,PVMind) = p_sel_target(j,1);
            PVM(4,PVMind) = p_sel_target(j,2);
            PVMind=PVMind+1;
        end
    else
        for j=1:size(p_sel_target,1)
            isFound = false;
            for k=1:PVMind-1
                if PVM(i*2-1,k) == p_sel_base(j,1) && ...
                   PVM(i*2,k) == p_sel_base(j,2)
                    PVM(i*2+1,k) = p_sel_target(j,1);
                    PVM(i*2+2,k)   = p_sel_target(j,2);
                    isFound=true;
                    break;
                end
            end
            if ~isFound
                PVM(i*2-1,PVMind) = p_sel_base(j,1);
                PVM(i*2,PVMind)   = p_sel_base(j,2);
                PVM(i*2+1,PVMind) = p_sel_target(j,1);
                PVM(i*2+2,PVMind) = p_sel_target(j,2);
                PVMind=PVMind+1;
            end
        end
    end
    
    if stopAfterFirstIteration
        return
    end
    
    isFirstIter = false;
end

%% plot PVM
if showPVM
    figure('name','Point-view matrix representation');
    %filter out the zeros
    PVM=PVM(:,1:find(PVM(end,:),1,'last'));
    imshow(PVM<1)
end
 


%% 5

if examplePVM
    % read matchview.txt
    f=fopen('PointViewMatrix.txt','r');
    PVM = fscanf(f,'%f');
    fclose(f);
    PVM = reshape(PVM,[215,202]);
    PVM = PVM';

end

figure('name','3D Structure created');
img_range = 1:2:nr_consec_imgs*2 -3;
img_x = 0;
[height, width] = size(PVM);
total_model = [];
last_transf = [];



for idx = img_range
    
    %find dense block
    non_zero = find(PVM(idx,:));
    non_zero = non_zero(non_zero>img_x);
    dense_block = [PVM(idx,non_zero);PVM(idx+1,non_zero);PVM(idx+2,non_zero);PVM(idx+3,non_zero)];
    if ~examplePVM
        img_x = size(non_zero,2);
    end
    %normalise
    mean_rows = mean(dense_block,2);
    dense_block = dense_block - mean_rows;
    % -> measurement matrix
    
    %SVD
    [U,W,V] = svd(dense_block);
    
    %reduce to rank 3
    U = U(:,1:3);
    W = W(1:3,1:3);
    V = V';
    V = V(1:3,:);
    
    %derive motion and shape/structure
    M = U*(W.^0.5);
    S = (W.^0.5)*V;

    %stitching
    if idx==img_range(1)
        total_model = [S];
    else
        %must have equal length/columns
        samples = min(size(S,2),size(last_transf,2));
        [d,Z,transform] = procrustes(last_transf(:,randsample(size(last_transf,2),samples )), S(:,randsample(size(S,2),samples ))  );
        total_model = [total_model Z];
    end
    
    last_transf = S;
 
end

fscatter3(total_model(1,:),total_model(2,:),total_model(3,:),total_model(3,:) );



