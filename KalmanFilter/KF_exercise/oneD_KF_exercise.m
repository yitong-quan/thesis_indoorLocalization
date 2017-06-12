clear all;
close all;
%% initiation
x_true = -0.37727;
%sigma: standard variant
sigma = 0.01;
%measurements: z
%z = sigma.*randn(50,1) + x_true;
load('z_measurements.mat');
z = ans;
% initial guess of the sate
x_0 = 0;
P_0 = 1;
% Q & R: process & measurement variance
Q = 1e-5;
R = sigma;
%% Kalman Filter
 % notation symbols please check Page 30 in 'An Intro to the Kalman Filter, G. Welch G. Bishop' 

% states vector and covariance: X(50,1) & P(50,1)
X = zeros(50,1);
P = zeros(50,1);
X = [x_0; X];
P = [P_0; P];
z = [0; z];
for i = 2:51
    % time update
    x_minus = X(i-1);
    P_minus = P(i-1) + Q;
    % meansurement update
        % Never use the inverse of a matrix to solve a linear system Ax=b with 
        % x=inv(A)*b, because it is slow and inaccurate.
        % Replace inv(A)*b with A\b, Replace b*inv(A) with b/A, replace A*inv(B)*C with A*(B\C).
        % HERE replace K = P(i-1)*inv(P(i-1) + R) with K = P(i-1)/(P(i-1) + R)
    K = P_minus/(P_minus + R);
    X(i) = x_minus + K*(z(i) - x_minus);  
    P(i) = (eye(size(K)) - K)*P_minus;
end
plot(z(2:end), '+');hold on;
plot(x_true*ones(50,1), 'g.');hold on;
plot(X(2:end));
%figure;
%plot(P(2:51));



