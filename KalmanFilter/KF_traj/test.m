%% 

% % test for plot color shade
% A = 1:0.1:20;
% B = sin (A);
% plot(A,B);

% test for randomly generate NaN in a matrix
clear all; close all;
numberOfMissing = 3;
A = ones(4,15);
ind = randi([1 size(A,1)] ,numberOfMissing,size(A,2));
figure;histogram(ind,size(ind,2));
for i = 1:size(A,2)
    for j = 1: numberOfMissing
        A(ind(j,i),i) = NaN;
    end
end    

% % test for randperm(Random permutation)
% clear all; close all;
% index1_NaN = randperm(6,3); % in this case, number from 1 to 6 will be all there, which is not what I want
% figure;histogram(index1_NaN,6);