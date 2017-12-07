clear
close
%% calculate the translateion matrix R and T

node_UWB = importdata('..\UWB\output_algo\nodesPositionLaserOptimal\nodePo.mat');
node_MoCap3 = importdata('..\mocap\nodespositionInMoCap3Nodes.mat');
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
[x,resnorm] = lsqnonlin(@myfun,x0,[],[],options);

M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];
node_MoCap3_RT = M(:,[1,2]) * node_MoCap3 + M(:,end);
plot(node_MoCap3_RT(1,:),node_MoCap3_RT(2,:),'xr');
title_string = ['theta=', num2str(wrapTo2Pi(x(1))),', t1=', num2str(x(2)),', t2=', num2str(x(3)),num2str(x(2)),', resnorm=', num2str(resnorm)];
title(title_string);
legend('node UWB(orig)', 'node MoCap 3(orig)', 'node MoCap 3(RT)');
% afterRT = R0 * nodes_optimal + t0;

function F = myfun(xx)
node_UWB = importdata('..\UWB\output_algo\nodesPositionLaserOptimal\nodePo.mat');
node_MoCap3 = importdata('..\mocap\nodespositionInMoCap3Nodes.mat');
node_MoCap3 = node_MoCap3/1000; %unit mm to m
M = [ [cos(xx(1)), -sin(xx(1)); sin(xx(1)), cos(xx(1))], xx(2:3)'];
afterRT = M(:,[1,2]) * node_MoCap3 + M(:,end);
dif = afterRT - node_UWB(:,[1,4,5]);
mis_dist = zeros(length(dif),1);
for i = 1:length(mis_dist)
    mis_dist(i) = norm(dif(:,i));
end
F = mis_dist;
end
