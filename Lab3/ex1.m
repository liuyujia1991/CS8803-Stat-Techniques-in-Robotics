%% Gaussian Process Regression with Gaussian RBF kernel
clear all
close all
clc

data = importdata('data/oakland_part3_an_rf.node_features');

data = data(data(:,5)==1004|data(:,5)==1400,:);
data = data(randperm(size(data,1)),:);

label = data(:,5);
feature = data(:,6:14);

train_num = 3000;
test_num = 500;
test_idx = 1:test_num;
train_idx = test_num+1:test_num+train_num;

omega = 10;
sigma = 10^-5;

tic;

Kd = rbf(feature(train_idx,:),omega) + eye(train_num)*sigma;
Kp = rbf(feature(test_idx,:),omega) + eye(test_num)*sigma;

toc;

Kdp = rbf(feature(test_idx,:),omega,feature(train_idx,:));

Mu = (Kdp/Kd)*label(train_idx);

tic;

labelp = 1400*(Mu(:)>=1202)+1004*(Mu(:)<1202);

toc;

accuracy = sum(labelp(:)==label(test_idx))/test_num;