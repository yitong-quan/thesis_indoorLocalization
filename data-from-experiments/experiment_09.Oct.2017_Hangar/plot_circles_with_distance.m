%%  nodes oder x2, x3, x1, x5, x6
    %     locations fixed: x1(0,0)  x3(+6,0) for v2
    %     locations fixed: x2(0,0)  x3(-6,0) for v1
    
% clear;
nodesNumber = 5;
if nodesNumber == 5
    distances2all_abs = importdata('dist_1st_tagOnCircleCenter.mat');
    positionOfNodes = importdata('nodePos_v2_by_determineNodesPositionBaseOnDistToEachOthers.mat');
end
% change the unit from m to mm
positionOfNodes = positionOfNodes*1000;
% change the oder to adapt to the nodes oder x2, x3, x1, x5, x6
distances2all_abs = [distances2all_abs(:,2), distances2all_abs(:,3), distances2all_abs(:,1), distances2all_abs(:,5), distances2all_abs(:,6)];
distances2all_abs = distances2all_abs';
% distances2all_abs = zeros(size(positionOfNodes, 2), size(traj, 2));
% for i = 1 : size(positionOfNodes, 2)
%     distances2each_xy = [traj(1:2, :) - repmat(positionOfNodes(:,i), 1, size(traj, 2))];
%     distances2each_abs = sqrt(distances2each_xy(1,:).^2 + distances2each_xy(2,:).^2);
%     distances2all_abs(i, :) = distances2each_abs;
% end
%axis square; %axis([-100 150 -200 200]);
%% make cross points of circles with Radius distances2all
% make circles
figure;
plot(positionOfNodes(1,:), positionOfNodes(2,:), '*');
hold on; % axis square; axis tight;
%plot the circle base on the optical measurements
optical_measurement = [4250, 5910, 4730, 4670, 5570];
for i = 1 : size(positionOfNodes, 2)
    h(i) = plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), ...
        optical_measurement(i), 0*pi, 2*pi); % 0, 2*pi); %
    %plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), distances2all_abs(i, 1), 0, 2*pi);
    %hold on;
end

for j = 1:size(distances2all_abs, 2)
    % h_1 = plot(traj(1,j), traj(2,j), 'ro'); %hold on;
    if nodesNumber == 5
        for i = 1 : size(positionOfNodes, 2)
            h(i) = plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), ...
                distances2all_abs(i, j), 0*pi, 2*pi); % 0, 2*pi); %
            %plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), distances2all_abs(i, 1), 0, 2*pi);
            %hold on;
        end
%         h(3) = plotCircle(positionOfNodes(1, 3), positionOfNodes(2, 3), ...
%             distances2all_abs(3, j), (i+4)*pi/2, (i+5)*pi/2+pi/6); % (i+2)*pi/2, (i+3)*pi/2);
    end
    axis square; axis tight;
    xlim([-5000 6000]); ylim([-1000 11000]);  
    pause(0.35);
    delete(h); %delete(h_1);
end
% make cross points 'x'

    
function h_circle = plotCircle(x,y,r,ang_start,ang_end)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=ang_start:0.01:ang_end; 
%ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
h_circle = plot(x+xp,y+yp);
end