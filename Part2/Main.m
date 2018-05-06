%% Add path variables and clear workspace
addpath(genpath('./Assignment_4'))
clear all; close all;
%% Load files

%TODO - eliminate points from background - how?
path = 'data/House/';
image = 'frame0000000%d.png';
im1name = sprintf([path,image],1);
im2name = sprintf([path,image],2);
im1 = imread(im1name);
im2 = imread(im2name);

%% Calculate Fundamental Matrix 

A = FundamentalMatrix(im1, im2);