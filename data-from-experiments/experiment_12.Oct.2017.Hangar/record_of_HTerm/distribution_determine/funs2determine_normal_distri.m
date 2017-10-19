% %% testing
% mean = 0.25;
% sigma = 0.3;
% R = normrnd(mean,sigma,[1,700]);

%% experiment data
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
%figure
subplot(2,2,1);
plot(R);
title('Subplot 1: R');
subplot(2,2,2);
cdfplot(R);
title('Subplot 2: cdf(R);');
subplot(2,2,3);
histogram(R);
title('Subplot 3: histogram(R);');

[h_lillie, p_lillie] = lillietest(R); % in the normal family
[h_jb, p_jb] = jbtest(R); % unknown mean and variance
[h_t, p_t] = ttest(R); % zero and unknown variance

[h_var, p_var] =vartest(R,sigma^2); % in the normal family
[h_z, p_z] =ztest(R, mean, sigma); % with mean m and a standard deviation sigma

[h_ks, p_ks] =kstest((R-mean)/sigma); % a standard normal distribution

if h_lillie == 1
    sprintf('---------lillie test rejects')
end
if h_jb == 1
    sprintf('--------- jb test rejects')
end
% if h_t == 1
%     sprintf('--------- t test rejects')
% end
% if h_var == 1
%     sprintf('--------- var test rejects')
% end
% if h_z == 1
%     sprintf('--------- z test rejects')
% end
% if h_ks == 1
%     sprintf('--------- ks test rejects')
% end