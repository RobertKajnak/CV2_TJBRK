%% Add path variables and clear workspace
addpath(genpath('./Assignment_4'))
clear all; close all;
%% Load files

%TODO - eliminate points from background - how?
path = 'data/House/';
image = 'frame0000000%d.png';
im1name = sprintf([path,image],1);
im2name = sprintf([path,image],9);
im1 = imread(im1name);
im2 = imread(im2name);

%% Get points of interest
%Specify number of sample points;
n = 50;

[p,pi] = InterestPoints(im1, im2, n, 1);

%%  3.1 Eight Point Algorithm
A = MakeA(p,pi);

[U,S,V] = svd(A);

%TODO is this math part correct?

[~,ind]=min(S(S~=0));

Fc = V(:,ind);

F = reshape(Fc,[3 3]);

[Uf,Df,Vf] = svd(F);

[~,ind] = min(Df(Df~=0));

Df(ind,ind)=0;

F = Uf*Df*Vf';

%% 3.2.1 Normalize 

p = [p,ones(n,1)];

p_hat = normalizedPi(p);

%% Normalize target points
pi = [pi,ones(n,1)];

pi_hat = normalizedPi(pi);

