%% mean and sigma comes from MATLAP>APP>Distribution_Fitting_Tool
% mean = 0.215651406;
% sigma = 0.179016874;
% R = normrnd(mean,sigma,[1,700]);

%% experiment data
% result
% XXXXX lillie test rejects
% ----------   jb test DONT rejects
% XXXXX  t test rejects
% XXXXX  var test rejects
% ----------   z test DONT rejects
% XXXXX  ks test rejects
%
estimated_posi_with_timeSt_EKF1 = importdata('../estimated_posi_with_timeSt_EKF_experi4.mat');
estimated_posi_with_timeSt_EKF2 = importdata('../estimated_posi_with_timeSt_EKF_experi3.mat');
estimated_posi_with_timeSt_EKF3 = importdata('../estimated_posi_with_timeSt_EKF_experi1.mat');
estimated_posi_with_timeSt_EKF = [estimated_posi_with_timeSt_EKF1, estimated_posi_with_timeSt_EKF2, estimated_posi_with_timeSt_EKF3];
location = estimated_posi_with_timeSt_EKF(1:2,:);
circle_center = importdata('../circleCenterPos_by_determineCircleCenterPositionBaseOnDistToEachOthers.mat');
loc2center = location - circle_center;
loc2center_norm = sqrt(sum(loc2center.^2,1));
disalignment = loc2center_norm - 2.5;
disalignment_trimed = disalignment;
disalignment_trimed(disalignment_trimed < -0.3) = [];
disalignment_trimed(disalignment_trimed > 0.8) = [];
R = disalignment_trimed;

mean = 0.215651406;
sigma = 0.179016874;

%% figure
subplot(2,2,1);
plot(R);
title('Subplot 1: R');
subplot(2,2,2);
cdfplot(R);
title('Subplot 2: cdf(R);');
subplot(2,2,3);
histogram(R);
title('Subplot 3: histogram(R);');

%% diff tests
[h_lillie, p_lillie] = lillietest(R); % in the normal family
[h_jb, p_jb] = jbtest(R); % unknown mean and variance
[h_t, p_t] = ttest(R); % zero and unknown variance

[h_var, p_var] =vartest(R,sigma^2); % comes from a normal distribution with variance v
[h_z, p_z] =ztest(R, mean, sigma); % with mean m and a standard deviation sigma

[h_ks, p_ks] =kstest((R-mean)/sigma); % a standard normal distribution

if h_lillie == 1
    disp('XXXXX lillie test rejects');
else
    disp('----------  lillie test DONT rejects');
end

if h_jb == 1
    disp('XXXXX  jb test rejects');
else
    disp('----------   jb test DONT rejects');
end

if h_t == 1
    disp('XXXXX  t test rejects');
else
    disp('----------   t test DONT rejects');
end

if h_var == 1
    disp('XXXXX  var test rejects');
else
    disp('----------   var test DONT rejects');
end

if h_z == 1
    disp('XXXXX  z test rejects');
else
    disp('----------   z test DONT rejects');
end

if h_ks == 1
    disp('XXXXX  ks test rejects');
else
    disp('----------   ks test DONT rejects');
end