
work_space_ekf_n = importdata('..\1_workspace_0.001_1.mat'); % all
work_space_ekf_calib = importdata('..\..\..\algo\self_calib\OUTPUT\1_allData_results_workspace.mat'); % all
work_space_ekf_s = importdata('..\40Hz\1_3_workspace_0.0008_6.5536_40Hz.mat'); % 152:218
x_n = work_space_ekf_n.X(:,152:218);
x_s = work_space_ekf_s.X;
figure; hold on;
plot(x_n(1,:),x_n(2,:),'k+-');
x_calib = work_space_ekf_calib.opt_tag_after_RRT(:,152:218); % 88:148 % 152:218
plot(x_calib(1,:),x_calib(2,:),'bx-');
node_calib = work_space_ekf_calib.opt_node_after_RRT;
plot(node_calib(1,:),node_calib(2,:),'bo');

%% velocity
v_n = x_n(3:4,:);
v_s = x_s(3:4,:);
dp_calib = diff(x_calib,1,2);
v_calib_x = dp_calib(1,:)'./work_space_ekf_n.time_diff(152:218-1);
v_calib_y = dp_calib(2,:)'./work_space_ekf_n.time_diff(152:218-1);
v_calib = [v_calib_x;v_calib_y];
MoCap_data_here = work_space_ekf_s.MoCap_data(min(work_space_ekf_s.near_idex):max(work_space_ekf_s.near_idex),7:9);
t_m = MoCap_data_here(:,3);
v_m_x = diff(MoCap_data_here(:,1))./diff(t_m );
v_m_x = movmean(v_m_x,100);
v_m_y = diff(MoCap_data_here(:,2))./diff(t_m );
v_m_y = movmean(v_m_y,100);

figure;hold on;
t_n = work_space_ekf_n.timeStamp(152:218);
plot(t_n-min(t_n), v_n(1,:),'k');
plot(t_n(1:end-1)-min(t_n), v_calib_x, 'b');
t_v = work_space_ekf_s.timeStamp;
h22 = plot(t_v-min(t_v), v_s(1,:), 'g');
h223 = plot(t_m(1:end-1)-min(t_m), v_m_x, 'r');


figure;hold on;
plot(t_n-min(t_n), v_n(2,:),'k');
plot(t_n(1:end-1)-min(t_n), v_calib_y, 'b');
h22 = plot(t_v-min(t_v), v_s(2,:), 'g');
h223 = plot(t_m(1:end-1)-min(t_m), v_m_y, 'r');
% 
% MoCap_data_here = work_space_ekf_s.MoCap_data(min(work_space_ekf_s.near_idex):max(work_space_ekf_s.near_idex),7:9);
% t_m = MoCap_data_here(:,3);
% v_m_x = diff(MoCap_data_here(:,1))./t_m(1:end-1);
% v_m_y = diff(MoCap_data_here(:,2))./t_m(1:end-1);


