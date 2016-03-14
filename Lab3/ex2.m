%% Bayesian Linear Regression

clear all
close all
clc

data = importdata('data/oakland_part3_am_rf.node_features');

data = data(data(:,5)==1100|data(:,5)==1103,:);
data = data(randperm(size(data,1)),:);

label = data(:,5);
feature = data(:,6:14);

train_num = 1000;
test_num = 1000;
train_idx = test_num+1:test_num+train_num;
test_idx = 1:test_num;

tic;

Mu = ones(size(feature,2),1);
gamma = 0.01;
Sigma = eye(size(feature,2));

P = inv(Sigma);
J = Sigma\Mu;

for i=train_idx
    J = label(i)*feature(i,:)'./(gamma^2) + J;
    P = feature(i,:)'*feature(i,:)./(gamma^2) + P;
end

toc;
tic;

Xstar = feature(test_idx,:);
Ystar = Xstar*(P\J);

labelp = 1103*(Ystar(:)>=1101.5)+1100*(Ystar(:)<1101.5);

toc;

accuracy = sum(labelp(:)==label(test_idx))/test_num;