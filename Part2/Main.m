%% Add path variables and clear workspace
addpath(genpath('./Assignment_4'))
clear all; close all;
%% Load files
path = 'data/House/';

files = dir(path);
M = length(files)-2;
images = cell(1,M);
for i=3:length(files)
    images{i-2} = files(i).name;
end

%TODO - eliminate points from background - how?
%% Load through all files
for i=1:M-1
    im1name = sprintf([path,images{i}],i);
    im2name = sprintf([path,images{i+1}],i+1);
    im1 = imread(im1name);
    im2 = imread(im2name);

    %% Get points of interest
    %Specify number of sample points;
    n = 50;

    [p, pi] = InterestPoints(im1, im2, n, 1);

    %%  3.1 Eight Point Algorithm
    A = MakeA(p,pi);

    F = MakeF(A);

    %% 3.2.1 Normalize source points

    [p_hat,T] = normalizedPi(p);

    %% 3.2.1 Normalize target points

    [pi_hat,T_prime] = normalizedPi(pi);

    %% 3.2.2 Make A Matrix

    A_hat = MakeA(p_hat,pi_hat);

    F_hat = MakeF(A_hat);

    F = T_prime'*F_hat*T;

    %% TODO 3.3

    %% 4. 
end