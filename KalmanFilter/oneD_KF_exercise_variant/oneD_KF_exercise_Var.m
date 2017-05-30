clear all;
close all;
%% initiation
x_true = -0.37727;
%sigma: standard variant
sigma = 0.01;
%measurements: z
z = sigma.*randn(100,1) + x_true;

% initial guess of the sate
x_0 = -0.3;
P_0 = 1;
% Q & R: process & measurement variance
Q = 1e-5;
R = sigma;
%% Kalman Filter
 % notation symbols please check Page 30 in 'An Intro to the Kalman Filter, G. Welch G. Bishop' 

% estimated states vector and covariance: X(length(z),1) & P(length(z),1)
X = zeros(length(z),1);
P = zeros(length(z),1);
X = [x_0; X];
P = [P_0; P];
z = [0; z];
for i = 2:length(z)
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
plot(x_true*ones(length(z),1), 'g.');hold on;
plot(X(2:end));
%figure;
%plot(P(2:51));
X_endPart = X(floor(size(X)*4/5) : end);
X_endPart_dirivation_max = max(X_endPart) - x_true;
X_endPart_dirivation_min = min(X_endPart) - x_true;
fprintf('last 1/5 states X converges to: \n dirivation_max =  %f, \n dirivation_min = %f\n',...
         X_endPart_dirivation_max, X_endPart_dirivation_min);



