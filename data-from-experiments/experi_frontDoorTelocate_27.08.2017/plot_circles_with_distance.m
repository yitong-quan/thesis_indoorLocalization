% clear;
nodesNumber = 4;
if nodesNumber == 3
    %distances2all_abs = importdata('3nodes_fixedTag_DataRemoveRowWith0.mat');
    distances2all_abs = importdata('3nodes_fixedTag_fullData.mat');
    positionOfNodes = [0,0,4340;0,-6360,0];
end
if nodesNumber == 4
    %distances2all_abs = importdata('movingO_DataRemoveRowWith0.mat');
    distances2all_abs = importdata('movingO_1_fullData.mat');
    %distances2all_abs = importdata('movingO_1_DataRemoveRowWith0.mat');
    %distances2all_abs = importdata('movingO_2_DataRemoveRowWith0.mat');
    positionOfNodes = [0,0,2500,4340;0,-6360,-6360,0];
end
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
hold on; % axis square; axis tight;

for j = 1:size(distances2all_abs, 2)
    % h_1 = plot(traj(1,j), traj(2,j), 'ro'); %hold on;
    if nodesNumber == 3
        plot(4340, -3000, '*'); % position of Tag
        for i = 1 : size(positionOfNodes, 2)-1
            h(i) = plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), ...
                distances2all_abs(i, j), (i+2)*pi/2, (i+3)*pi/2); % 0, 2*pi); %
            %plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), distances2all_abs(i, 1), 0, 2*pi);
            %hold on;
        end
        h(3) = plotCircle(positionOfNodes(1, 3), positionOfNodes(2, 3), ...
            distances2all_abs(3, j), (i+4)*pi/2, (i+5)*pi/2+pi/6); % (i+2)*pi/2, (i+3)*pi/2);
    end
    if nodesNumber == 4
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
    xlim([0 5000]); ylim([-7000 0]);
    pause(0.35);
    delete(h); %delete(h_1);
end
%axis([-50 100 -50 100]);
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