% clear all;
% close all;
%% import measuremnets data from recording
% TODO load data
measurements_data = importdata('..\..\trajectory\goodTraj01\noisy_measuremnts_data.mat');
%% initiation
X_0 = [0 0 1 1]';
P_0 = ones(4,4); % choose to be all one, a litle too big, but it should converge at the end if the KF work 
% sampling time interval
dt = 2/3;
% state transition model matrix A
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
% control-input model matrix B
G = [dt^2/2*eye(2);dt*eye(2)];
sigma = 1.0;% shoule be relative to the value of 'noiseLevelForMeasurements.mat' in folder 'goodTraj01'
B = G * sigma * randn(2,1);
% Q & R: process & measurement variance
Q = 1e-5; % choose a small but non-0 value, more flexible in 'tuning' the filter
R = sigma; % shoule be relative to the value of 'noiseLevelForMeasurements.mat' in folder 'goodTraj01'

%% Kalman Filter
 % notation symbols please check Page 30 in 'An Intro to the Kalman Filter, G. Welch G. Bishop' 
X = zeros(4, size(measurements_data,2));
P = zeros(4, size(measurements_data,2));

X(:, 1) = X_0;
P(:, 1) = P_0;
z = measurements_data;
% for loop, and plot position on map
for i = 2:size(measurements_data,2)
    
end