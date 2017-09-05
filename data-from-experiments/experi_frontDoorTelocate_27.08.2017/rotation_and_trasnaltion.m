x_optimal = importdata('movingO_bestResult_X_value.mat');
nodes_optimal = x_optimal(:,[53:56]);
nodes_true = [0,0,2.5,4.34;6.36,0,0,6.36];
R0 = zeros(2);
t0 = zeros(2,1);
M0 = [R0, t0];
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
[M,resnorm] = lsqnonlin(@myfun,M0,[],[],options);
figure;
plot(nodes_true(1,:), nodes_true(2,:), '-or');
hold on;
plot(nodes_optimal(1,:), nodes_optimal(2,:), '-og');
nodes_afterRT = M(:,[1,2]) * nodes_optimal + M(:,end);
plot(nodes_afterRT(1,:), nodes_afterRT(2,:), '-ob');
legend('true position(nodes)', 'estimated position before rotation(nodes)', 'estimated position after rotation(nodes)');

figure;
plot(nodes_true(1,:), nodes_true(2,:), '-or');
hold on;
plot(x_optimal(1,[1:52]), x_optimal(2,[1:52]),'g-*', nodes_optimal(1,:), nodes_optimal(2,:),'-og');
tagNodes_afterRT = M(:,[1,2]) * x_optimal + M(:,end);
plot(tagNodes_afterRT(1,[1:52]), tagNodes_afterRT(2,[1:52]),'b-*', tagNodes_afterRT(1,[53:56]), tagNodes_afterRT(2,[53:56]),'b-o');
legend('true position(nodes)', 'estimated position before rotation(tag)', 'estimated position before rotation(nodes)', 'estimated position after rotation(tag)', 'estimated position after rotation(nodes)');

M,resnorm

% afterRT = R0 * nodes_optimal + t0;

function F = myfun(M)
x_optimal = importdata('movingO_bestResult_X_value.mat');
nodes_optimal = x_optimal(:,[53:56]);
nodes_true = [0,0,2.5,4.34;6.36,0,0,6.36];
afterRT = M(:,[1,2]) * nodes_optimal + M(:,end);
dif = afterRT - nodes_true;
mis_dist = zeros(length(dif));
for i = 1:length(mis_dist)
    mis_dist(i) = norm(dif(:,i));
end
F = mis_dist;
end