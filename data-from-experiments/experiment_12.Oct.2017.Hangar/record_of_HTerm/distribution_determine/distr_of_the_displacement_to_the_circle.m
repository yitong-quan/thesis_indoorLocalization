estimated_posi_with_timeSt_EKF1 = importdata('../estimated_posi_with_timeSt_EKF_experi4.mat');
estimated_posi_with_timeSt_EKF2 = importdata('../estimated_posi_with_timeSt_EKF_experi3.mat');
estimated_posi_with_timeSt_EKF3 = importdata('../estimated_posi_with_timeSt_EKF_experi1.mat');
estimated_posi_with_timeSt_EKF = [estimated_posi_with_timeSt_EKF1, estimated_posi_with_timeSt_EKF2, estimated_posi_with_timeSt_EKF3];

location = estimated_posi_with_timeSt_EKF(1:2,:);
circle_center = importdata('../circleCenterPos_by_determineCircleCenterPositionBaseOnDistToEachOthers.mat');
loc2center = location - circle_center;
loc2center_norm = sqrt(sum(loc2center.^2,1));
figure; plot(loc2center_norm,'-+');title('distance to center');
disalignment = loc2center_norm - 2.5;
figure; plot(disalignment,'-+');title('disalignment to radius');
disalignment_trimed = disalignment;
disalignment_trimed(disalignment_trimed < -0.4) = [];
disalignment_trimed(disalignment_trimed > 0.8) = [];
figure; plot(disalignment_trimed,'-+');title('disalignmentTrimed to radius');
for i = 1:10
    figure; hist(disalignment_trimed, min(disalignment_trimed):0.01*i:max(disalignment_trimed));title('disalignmentTrimed histogram');
end  

% %figure; hist(disalignment);title('disalignment histogram');
% for i = 1:10
%     figure; hist(disalignment, min(disalignment):0.01*i:max(disalignment));title('disalignment histogram');
% end    
% % subset = disalignment(55:240);
% % figure; hist(subset,min(subset):0.05:max(subset));title('disalignmentSubset histogram');

