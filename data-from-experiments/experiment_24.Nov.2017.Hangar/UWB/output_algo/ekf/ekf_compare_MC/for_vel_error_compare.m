%children = get(gca, 'children')
%children(1)
%children(4)
%delete(children(4));
%children(7)
%children(6)
%children = get(gca, 'children')
%children(6)
%children(5)
%children = get(gca, 'children')
%children(5)
children = get(gca, 'children')
children(1)
children(1).XData
children_4 = get(gca, 'children')
children_4(1)
children_4(2)
x=children_4(2).XData;
x(end)
figure4
figure
childen
children
children_2
%%
vel_EKF_MC1= children_2(1);
vel_EKF_LWCR= children_2(2);
vel_diff_EKF_LWCR=vel_EKF_LWCR;
vel_diff_EKF_MC1=vel_EKF_MC1;
children_4
vel_EKF-MC2 = children_4(1)
vel_EKF_MC2 = children_4(1)
vel_OM = children_4(2)
vel_EKF_MC1 = children_4(3)
%children_4(4)
%children_4
%children_4(4)
vel_EKF_true = children_4(4);
timeS = vel_diff_EKF_MC1.XData;
figure
%plot(timeS,(vel_EKF_MC1-vel_EKF_true))
plot(timeS,(vel_EKF_MC1.YData-vel_EKF_true.YData))
hold on
plot(timeS,(vel_diff_EKF_MC1.YData))
children_3 = get(gca, 'children')
vel_EKF_MC1= children_3(1);
vel_EKF_LWCR= children_3(2);
vel_diff_EKF_LWCR=vel_EKF_LWCR;
vel_diff_EKF_MC1=vel_EKF_MC1;
plot(timeS,(vel_diff_EKF_MC1.YData),'+')
figure; hold on
plot(timeS,(vel_diff_EKF_MC1.YData),'k')
plot(timeS,(vel_diff_EKF_LWCR.YData),'m')
plot(timeS,(vel_OM.YData-vel_EKF_true.YData),'b')
timeStamp_for_MC2_ = timeStamp_for_MC2- timeStamp_for_MC2(1);
h1=plot(timeStamp_for_MC2_,(vel_diff_EKF_MC1.YData),'k')
vel_diff_EKF_MC1.YData;
vel_diff_EKF_MC2.YData;
vel_EKF_MC2.YData;
vel_diff_EKF_MC2 = vel_EKF_MC2 -
r=repmat(vel_EKF_true,5,1)';
r=repmat(vel_EKF_true.YData,5,1)';
r1=r(:)';
r1=r(:);
r=r';
r1=r(:);
vel_EKF_true_for_MC2 = r2;
vel_EKF_true_for_MC2 = r1;
vel_diff_EKF_MC2 = vel_EKF_MC2-vel_EKF_true_for_MC2;
vel_diff_EKF_MC2 = vel_EKF_MC2.YData-vel_EKF_true_for_MC2.YData;
vel_diff_EKF_MC2 = vel_EKF_MC2.YData-vel_EKF_true_for_MC2;
h1=plot(timeStamp_for_MC2_,vel_diff_EKF_MC2,'g')
vel_diff_EKF_MC2 = vel_EKF_MC2.YData-vel_EKF_true_for_MC2';
h1=plot(timeStamp_for_MC2_,vel_diff_EKF_MC2,'g')
timeStamp_for_MC2_ = timeStamp_for_MC2_(1:end-5);
h1=plot(timeStamp_for_MC2_,vel_diff_EKF_MC2,'g')
vel_diff_EKF_MC1_all_data = vel_diff_EKF_MC1;
vel_diff_EKF_MC1 = vel_diff_EKF_MC1_all_data.YData;
vel_diff_EKF_LWCR_all_data = vel_diff_EKF_LWCR;
vel_diff_EKF_LWCR = vel_diff_EKF_LWCR_all_data.YData;
vel_diff_OM=vel_OM.YData-vel_EKF_true.YData;
figure;h = histogram(vel_diff_EKF_MC2,'Normalization','probability')
2
hold on;h = histogram(vel_diff_EKF_MC2,'Normalization','probability');
figure;histo_MC2 = histogram(vel_diff_EKF_MC2,'Normalization','probability')
hold on;histo_MC1 = histogram(vel_diff_EKF_MC1,'Normalization','probability')
hold on;histo_LWCR = histogram(vel_diff_EKF_LWCR,'Normalization','probability')
hold on;histo_OM = histogram(vel_diff_OM,'Normalization','probability')


%% 
clear
EKF_LWCR_data = importdata('3_1_workspace_0.001_1_weighted.mat');
EKF_SM = importdata('3_1_workspace_0.0008_6.5536_40Hz.mat');
EKF_MM = importdata('3_1_workspace_0.001_1.mat');
OM = importdata('3_1_group_results_workspace.mat');
real_X = EKF_MM.real_X;
OM_X =OM.opt_tag_after_RRT;
figure; plot(OM_X(1,:),OM_X(2,:),'-*')
EKF_MM_X =EKF_MM.X;
EKF_SM_X =EKF_SM.X;
EKF_LWCR_X =EKF_LWCR_data.X;
EKF_MM_mis_pos =EKF_MM.mis_pos;
EKF_SM_mis_pos =EKF_SM.mis_pos;
EKF_LWCR_mis_pos =EKF_LWCR.mis_pos;
EKF_LWCR_mis_pos =EKF_LWCR_data.mis_pos;
OM_mis_pos =OM_X-real_X(1:2,1:82);
figure; hold on;
histo_MC1 = histogram(EKF_MM_mis_pos(1,:),'Normalization','probability')
histo_MC2 = histogram(EKF_SM_mis_pos(1,:),,'Normalization','probability')
histo_MC2 = histogram(EKF_SM_mis_pos(1,:),'Normalization','probability')
histo_OM = histogram(OM_mis_pos(1,:),'Normalization','probability')
delete(histo_OM)
histo_LWCR = histogram(EKF_LWCR_mis_pos(1,:),'Normalization','probability')
real_X_for_OM = real_X;
real_X_for_OM(:,3) =[];
real_X_for_OM = real_X;
real_X_for_OM(:,63) =[];
real_X_for_OM(:,57) =[];
real_X_for_OM(:,35) =[];
real_X_for_OM(:,3) =[];
OM_mis_pos =OM_X-real_X_for_OM;
histo_OM = histogram(OM_mis_pos(1,:),'Normalization','probability')
figure; hold on;
histo_MC1 = histogram(EKF_MM_mis_pos(2,:),'Normalization','probability')
histo_MC2 = histogram(EKF_SM_mis_pos(2,:),'Normalization','probability')
histo_OM = histogram(OM_mis_pos(2,:),'Normalization','probability')
histo_LWCR = histogram(EKF_LWCR_mis_pos(2,:),'Normalization','probability')
