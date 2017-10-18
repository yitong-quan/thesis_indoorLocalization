estimated_posi_with_timeSt_EKF1 = importdata('estimated_posi_with_timeSt_EKF_experi4.mat');
estimated_posi_with_timeSt_EKF2 = importdata('estimated_posi_with_timeSt_EKF_experi3.mat');
estimated_posi_with_timeSt_EKF = [estimated_posi_with_timeSt_EKF1, estimated_posi_with_timeSt_EKF2];

location = estimated_posi_with_timeSt_EKF(1:2,:);
circle_center = importdata('circleCenterPos_by_determineCircleCenterPositionBaseOnDistToEachOthers.mat');
loc2center = location - circle_center;
loc2center_norm = sqrt(sum(loc2center.^2,1));
figure; plot(loc2center_norm,'-+');title('distance to center');
disalignment = loc2center_norm - 2.5;
figure; plot(disalignment,'-+');title('disalignment to radius');
%figure; hist(disalignment);title('disalignment histogram');
for i = 1:10
    figure; hist(disalignment, min(disalignment):0.01*i:max(disalignment));title('disalignment histogram');
end    
% subset = disalignment(55:240);
% figure; hist(subset,min(subset):0.05:max(subset));title('disalignmentSubset histogram');

