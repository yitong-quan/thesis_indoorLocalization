% results from this algo is not used yet
clear
%% calculate the translateion matrix R and T

node_UWB = importdata('..\UWB\output_algo\nodesPositionLaserOptimal\nodePo.mat');
node_MoCap3 = importdata('..\mocap\nodespositionInMoCap3Nodes.mat');
idx_alighment_UWB_MC = importdata('idx_alighment_UWB_MC_exp3.mat');

tag_uwb = importdata('..\UWB\data\data_t_dist_3_sq_t.mat');
tag_mc = importdata('..\mocap\afterRTtoUWB\cortex_json7_sq_RT2UWB.mat');
idx_alighment_UWB_MC = importdata('idx_alighment_UWB_MC_exp3.mat');
tag_uwb_trimmed = tag_uwb((2:39),[4,5]);
tag_mc_trimmed = tag_mc(idx_alighment_UWB_MC(282,319),[7,8]);

node_MoCap3 = node_MoCap3/1000; %unit mm to m
plot(node_UWB(1,:),node_UWB(2,:),'ob');
hold on;
plot(node_MoCap3(1,:),node_MoCap3(2,:),'*g');
xlabel({'x / m'});
ylabel({'y / m'});
daspect([10,10,10]);

% M0 = [ [cos(theta), -sin(theta); sin(theta), cos(theta)], [t1,t2]'];
x0 = 100*rand(1,3); % x = [theta, t1, t2]
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
resnorm_last = inf;
RESNORM = [];
x_last = x0;
for ii = 1:10
    [x,resnorm] = lsqnonlin(@myfun,x0,[],[],options);
    
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm;
        RESNORM =  [RESNORM, resnorm];    
    end
    x0 = x + 10*rand(size(x_opt));
end    

M = [ [cos(x_opt(1)), -sin(x_opt(1)); sin(x_opt(1)), cos(x_opt(1))], x_opt(2:3)'];
node_MoCap3_RT = M(:,[1,2]) * node_MoCap3 + M(:,end);
plot(node_MoCap3_RT(1,:),node_MoCap3_RT(2,:),'xr');
title_string = ['theta=', num2str(wrapTo2Pi(x_opt(1))),', t1=', num2str(x_opt(2)),', t2=', num2str(x_opt(3)), ', resnorm=', num2str(resnorm_opt)];
title(title_string);
legend('node UWB(orig)', 'node MoCap 3(orig)', 'node MoCap 3(RT)');
% afterRT = R0 * nodes_optimal + t0;

figure; plot(RESNORM, '-*');
RESNORM

function F = myfun(xx)
node_UWB = importdata('..\UWB\output_algo\nodesPositionLaserOptimal\nodePo.mat');
node_MoCap3 = importdata('..\mocap\nodespositionInMoCap3Nodes.mat');
tag_uwb = importdata('..\UWB\data\data_t_dist_3_sq_t.mat');
tag_mc = importdata('..\mocap\afterRTtoUWB\cortex_json7_sq_RT2UWB.mat');
idx_alighment_UWB_MC = importdata('idx_alighment_UWB_MC_exp3.mat');
tag_uwb_trimmed = tag_uwb((2:39),[4,5]);
tag_mc_trimmed = tag_mc(idx_alighment_UWB_MC(282,319),[7,8]);
node_MoCap3 = node_MoCap3/1000; %unit mm to m
M = [ [cos(xx(1)), -sin(xx(1)); sin(xx(1)), cos(xx(1))], xx(2:3)'];
%afterRT = M(:,[1,2]) * node_MoCap3 + M(:,end);
afterRT = M(:,[1,2]) * [node_MoCap3;tag_mc_trimmed'] + M(:,end);
%dif = afterRT - node_UWB(:,[1,4,5]);
dif = afterRT - [node_UWB(:,[1,4,5]);tag_uwb_trimmed'];
mis_dist = zeros(length(dif),1);
for i = 1:length(mis_dist)
    mis_dist(i) = norm(dif(:,i));
end
F = mis_dist;
end
