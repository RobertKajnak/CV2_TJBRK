%% Add path variables and clear workspace
run('.\vlfeat\toolbox\vl_setup');
clear all; close all;
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
%% Initialize some variables
isFirstIter = true;
%TODO - do dynamic reallocation within loop
maxExpectedFeatures=2000;
PVM=zeros((M-1)*2,maxExpectedFeatures);
showSift = false;
showEpipolar = false;

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
    [p_base, p_target] = InterestPoints(f1,f2,matches,30,showSift,im1,im2);
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
    
    p_base_prime = (T_prime^-1*p_base_hat')';
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
        drawEpipolar(F,p_base(1:8,:),p_target(1:8,:),im1,im2,'Epipolar lines using simple eight-point algorithm');

        %normalized eight-point
        drawEpipolar(F_prime,p_base(1:8,:),p_target(1:8,:),im1,im2,'Epipolar lines using normalized eight-point algorithm');

        %normalized RANSACed eight-point
        drawEpipolar(F_prime_rans,p_base_rans(1:8,:),p_target_rans(1:8,:),im1,im2,['Epipolar lines using eight-point algoirthm augmented by '...
                'normalization and RANSAC point selection']);
    end

  %% 4.
    p_sel_base = p_base;
    p_sel_target = p_target;
    %if first iteration add all points from first two images; these are
    %guaranteed to be matches
    if isFirstIter
        for j=1:size(p_sel_base,1)
            PVM(1,PVMind) = p_sel_base(j,1);
            PVM(2,PVMind) = p_sel_base(j,2);
            
            PVM(3,PVMind) = p_sel_target(j,1);
            PVM(4,PVMind) = p_sel_target(j,2);
            %PVMind keeps track of the last inserded column
            PVMind=PVMind+1;
        end
    else
        for j=1:size(p_sel_target,1)
            isFound = false;
            %if a point is found the lines pertaining to the next image
            %are filled.
            for k=1:PVMind-1
                if PVM(i*2-1,k) == p_sel_base(j,1) && ...
                   PVM(i*2,k) == p_sel_base(j,2)
                    PVM(i*2+1,k) = p_sel_target(j,1);
                    PVM(i*2+2,k)   = p_sel_target(j,2);
                    isFound=true;
                    break;
                end
            end
            %if the point is missing a new column is introduced, adding
            %both current and previous image's feature's coordinates to it
            if ~isFound
                PVM(i*2-1,PVMind) = p_sel_base(j,1);
                PVM(i*2,PVMind)   = p_sel_base(j,2);
                PVM(i*2+1,PVMind) = p_sel_target(j,1);
                PVM(i*2+2,PVMind) = p_sel_target(j,2);
                PVMind=PVMind+1;
            end
        end
    end
    isFirstIter = false;
end

%% plot PVM
figure('name','Point-view matrix representation');
%filter out the zeros
PVM=PVM(:,1:find(PVM(end,:),1,'last'));
imshow(PVM<1)


%% Alternative visualization method
% return
% %% read matchview.txt
% f=fopen('PointViewMatrix.txt','r');
% PVM = fscanf(f,'%f');
% fclose(f);
% 
% PVM = reshape(PVM,[215,202]);
% PVM = PVM';

%% testcase
% figure;
% imshow(imread(sprintf([path,images{1}],1)));
% hold on;
% for i=1:215
%     color = rand(1,3);
% 
%     for j = 1:2:101
%         color=color*.98;
%         if PVM(j,i)~=0 && PVM(j+1,i) ~=0
%             plot(PVM(j,i),PVM(j+1,i),'x','color',color)
%         end
%     end
% end


