% results from this algo is not used yet
clear
%% calculate the translateion matrix R and T
figure;

tag_uwb = importdata('..\UWB\output_algo\ekf\traj3_Xestimated.mat');
tag_uwb = tag_uwb([1,2],50:end);
tag_mc = importdata('..\UWB\output_algo\ekf\Xreal_RRT_nodeOnly\traj3RRT_nodesOnly_Xreal.mat');
tag_mc = tag_mc(:,50:end);

plot(tag_uwb(1,:),tag_uwb(2,:),'ob');
hold on;
plot(tag_mc(1,:),tag_mc(2,:),'*g');
xlabel({'x / m'});
ylabel({'y / m'});
daspect([10,10,10]);

% M0 = [ [cos(theta), -sin(theta); sin(theta), cos(theta)], [t1,t2]'];
x0 = 100*rand(1,3); % x = [theta, t1, t2]
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
resnorm_last = inf;
RESNORM = [];
x_last = x0;
for ii = 1:20
    [x,resnorm] = lsqnonlin(@myfun,x0,[],[],options);
    
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm;
        RESNORM =  [RESNORM, resnorm];    
    end
    x0 = x + 20*rand(size(x_opt));
end    

M = [ [cos(x_opt(1)), -sin(x_opt(1)); sin(x_opt(1)), cos(x_opt(1))], x_opt(2:3)'];
tag_mc_RT = M(:,[1,2]) * tag_mc + M(:,end);
plot(tag_mc_RT(1,:),tag_mc_RT(2,:),'xr');
title_string = ['theta=', num2str(wrapToPi(x_opt(1))),', t1=', num2str(x_opt(2)),', t2=', num2str(x_opt(3)), ', resnorm=', num2str(resnorm_opt)];
title(title_string);
legend('tag UWB(orig)', 'tag MoCap 3(orig)', 'tag MoCap 3(RT)');
% afterRT = R0 * nodes_optimal + t0;

figure; plot(RESNORM, '-*');
RESNORM

function F = myfun(xx)
tag_uwb = importdata('..\UWB\output_algo\ekf\traj3_Xestimated.mat');
tag_uwb = tag_uwb([1,2],50:end);
tag_mc = importdata('..\UWB\output_algo\ekf\Xreal_RRT_nodeOnly\traj3RRT_nodesOnly_Xreal.mat');
tag_mc = tag_mc(:,50:end);

M = [ [cos(xx(1)), -sin(xx(1)); sin(xx(1)), cos(xx(1))], xx(2:3)'];
%afterRT = M(:,[1,2]) * node_MoCap3 + M(:,end);
afterRT = M(:,[1,2]) * tag_mc + M(:,end);
%dif = afterRT - node_UWB(:,[1,4,5]);
dif = afterRT - tag_uwb;
mis_dist = zeros(length(dif),1);
for i = 1:length(mis_dist)
    mis_dist(i) = norm(dif(:,i));
end
F = mis_dist;
end
