%% 
%   assuming one of the r possible known fixed models(with differents R & Q)
%   is the correct model.
%   each model i has its weight p_i

%% Unfortunately the result is not good , the 'correct' values for Q AND P should be
%  Q: 1e-5 AND P: sigma. but if 1e-5/25 and sigma/25 are given as a option,
%  this pair will be choosen as the iterations go further.
%% 
clear all; close all;
%% initiation
% number of the possible known fixed models
r = 4;
x_true = -0.37727;
%sigma: standard variant
sigma = 0.01;
%measurements: z
z = sigma.*randn(500,1) + x_true;
% initial guess of the sate. r possible states for r possible models
x_0 = -0.3*ones(1,r); P_0 = 1*ones(1,r);
% initial guess of the weights
p = 1/r * ones(length(z),r);
f = ones(1,r); % f for calculation of p
% initial guess of process & measurement variance: Q & R
Q = [rand(1, r-3), 1e-5, 1e-10, 1e-5/25];  % correct value: 1e-5
R = [rand(1, r-3), sigma, 1e-10, sigma/25]; % correct value: sigma
%% Kalman Filter
 % notation symbols please check Page 30 in 'An Intro to the Kalman Filter, G. Welch G. Bishop' 

% estimated states vector and covariance: X(length(z),1) & P(length(z),1)
X = zeros(length(z),r);
P = zeros(length(z),r);
X = [x_0; X];
P = [P_0; P];
z = [0; z];
p = [1/r*ones(1,r); p];
K = ones(1,r); % kalman gain
% initiate the final estimate state and covariance X_F and P_F
X_F = zeros(length(z),1);
P_F = zeros(length(z),1);
% begin for loop for iteration
for i = 2:length(z)
    % time update
    x_minus = X(i-1,:);
    P_minus = P(i-1,:) + Q;
    % meansurement update
        % Never use the inverse of a matrix to solve a linear system Ax=b with 
        % x=inv(A)*b, because it is slow and inaccurate.
        % Replace inv(A)*b with A\b, Replace b*inv(A) with b/A, replace A*inv(B)*C with A*(B\C).
        % HERE replace K = P(i-1)*inv(P(i-1) + R) with K = P(i-1)/(P(i-1) + R)
%            K = P_minus/(P_minus + R);
%            X(i) = x_minus + K*(z(i) - x_minus);  
%            P(i) = (eye(size(K)) - K)*P_minus;
    %%%% step a&b &c(update K&P)
    C = P_minus + R;
    for j = 1:length(C)
        % e2power = -1/2 * (z(i) - x_minus)'*inv(C(i))*(z(i) - x_minus);
        e2power = -1/2 * (z(i) - x_minus(j))'*(C(j)\(z(i) - x_minus(j)));        
        % f: probability of the likelihood of a measurement
        %f(j) = 1/(sqrt( (2*pi)^r * abs(C(j))) ) * exp(e2power);
        f(j) = 1/(sqrt( (2*pi*abs(C(j)))^4 )) * exp(e2power);        
        % p: probability of each model is the correct one
        p(i,j) = f(j)*p(i-1,j);
        %%%% step c
        K(j) = P_minus(j)/(P_minus(j) + R(j));
        P(i,j) = (eye(size(K(j))) - K(j))*P_minus(j);
    end
    p(i,:) = p(i,:)/sum(p(i,:)); % normalize p 
    %%%% step c(update X)
    X(i,:) = x_minus + K.*(z(i)*ones(1,r) - x_minus);
    X_F(i) = sum(p(i,:)*X(i,:)');
    eps = X_F(i)*ones(1,r) - X(i,:);
    P_F(i) = p(i,:)*(P(i,:) + eps.*eps)';
        
end
plot(z(2:end), '+');hold on;
plot(x_true*ones(length(z),1), 'g.');hold on;
plot(X_F(2:end));
figure;
plot(p,'*');    
legend('1','2','3','4');

%figure;
%plot(P(2:51));
% X_endPart = X(floor(size(X)*4/5) : end);
% X_endPart_dirivation_max = max(X_endPart) - x_true;
% X_endPart_dirivation_min = min(X_endPart) - x_true;
% fprintf('last 1/5 states X converges to: \n dirivation_max =  %f, \n dirivation_min = %f\n',...
%          X_endPart_dirivation_max, X_endPart_dirivation_min);



