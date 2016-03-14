%% Kernel Logistic Regression
clear all
close all
clc

data = importdata('data/oakland_part3_an_rf.node_features');
data = data(randperm(size(data,1)),:);

feature = data(:,6:14);

train_num = 5000;
test_num = 5000;
test_idx = 1:test_num;
train_idx = test_num+1:test_num+train_num;

sigma = 1;
eta = 0.01;
lambda = 1;

tic;

Kd = rbf(feature(train_idx,:),sigma);
Kdp = rbf(feature(test_idx,:),sigma,feature(train_idx,:));

class = [1004 1100 1103 1200 1400];
alpha = zeros(train_num, 5);

for i=1:5
    
    label = 2*(data(:,5)==class(i)) - 1;
    alpha(:,i) = zeros(train_num,1);
    gamma = zeros(train_num,1);
    old_Loss = 5;
    Loss = 0;

    while abs(old_Loss-Loss) >= 0.1
    old_Loss = Loss;
    gamma = -label(train_idx).*exp(-label(train_idx).*(Kd*alpha(:,i)))./(1+exp(-label(train_idx).*(Kd*alpha(:,i))));
    alpha(:,i) = (1-eta*lambda)*alpha(:,i) - eta*gamma; 
    Loss = sum(log(1+exp(-label(train_idx).*(Kd*alpha(:,i)))))+lambda/2*(alpha(:,i)'*alpha(:,i));
    end
    
end

toc;
tic;

Mu = 1./(1+exp(-Kdp*alpha));
[prob labelp] = max(Mu');
labelp = labelp';

label = 1*(data(:,5)==1004)+2*(data(:,5)==1100)+3*(data(:,5)==1103)+4*(data(:,5)==1200)+5*(data(:,5)==1400);

toc;

accuracy = sum(labelp(:)==label(test_idx))/test_num;