%clear; close all;
%% import data with a good trajectory. plot positions and velocity
traj = importdata('goodTraj01\position01.mat');
% plot velocities in x&y directions and the differenet of these 2 and the
% absolute velocity
plot(traj(3,:), 'r.');hold on; plot(traj(4,:), '.');hold on; 
plot(traj(3,:) - traj(4,:), 'y*');hold on;
plot(sqrt((traj(3,:).^2 + traj(4,:).^2)), 'g-');
legend('v_x','v_y','diff(v_x, v_y)','absolute(v)');
% plot positions
figure();
plot(traj(1,:), traj(2,:), '.'); legend('positions'); hold on;

%% fixed nodes sensors position and plot 'O'
positionOfNodes = [-50 -50; 100 -50; 100 100; -50 100]';
plot(positionOfNodes(1,:),positionOfNodes(2,:), 'O');
hold on;

%% calulate the distsance between positions and nodes: 'distances2all_abs'
distances2all_abs = zeros(size(positionOfNodes, 2), size(traj, 2));
for i = 1 : size(positionOfNodes, 2)
    distances2each_xy = [traj(1:2, :) - repmat(positionOfNodes(:,i), 1, size(traj, 2))];
    distances2each_abs = sqrt(distances2each_xy(1,:).^2 + distances2each_xy(2,:).^2);
    distances2all_abs(i, :) = distances2each_abs;
end
%axis square; %axis([-100 150 -200 200]);
%% make cross points of circles with Radius distances2all
% make circles
for j = 1:size(traj, 2)
    h_1 = plot(traj(1,j), traj(2,j), 'ro'); %hold on;
    for i = 1 : size(positionOfNodes, 2)
        h(i) = plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), ...
            distances2all_abs(i, j), (i-1)*pi/2, i*pi/2);
        %plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), distances2all_abs(i, 1), 0, 2*pi);
        %hold on;
    end    
    axis square; axis tight;
    pause(0.1);
    delete(h); delete(h_1);
end
%axis([-50 100 -50 100]);
% make cross points 'x'
