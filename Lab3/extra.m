%% Gaussian Process Regression with SAM kernel
clear all
close all
clc

data = importdata('data/oakland_part3_am_rf.node_features');

data = data(data(:,5)==1100|data(:,5)==1103,:);
data = data(randperm(size(data,1)),:);

label = data(:,5);
feature = data(:,6:14);

train_num = 1000;
test_num = 500;
test_idx = 1:test_num;
train_idx = test_num+1:test_num+train_num;

omega = 10;
sigma = 10^-5;

tic;

Kd = sam(feature(train_idx,:),omega) + eye(train_num)*sigma;
Kp = sam(feature(test_idx,:),omega) + eye(test_num)*sigma;

toc;

Kdp = sam(feature(test_idx,:),omega,feature(train_idx,:));
Mu = (Kdp/Kd)*label(train_idx);

tic;

labelp = 1103*(Mu(:)>=1101.5)+1100*(Mu(:)<1101.5);

toc;

accuracy = sum(labelp(:)==label(test_idx))/test_num;
