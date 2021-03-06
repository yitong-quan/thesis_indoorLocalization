%%  nodes oder x2, x3, x1, x5, x6
    %     locations fixed: x1(0,0)  x3(+6,0) for v2
    %     locations fixed: x2(0,0)  x3(-6,0) for v1
    
% clear;
experiment = 4; % [1 2 3 4]

nodesNumber = 5;
circle_center = [1000, 4750];
circle_angle = [0:pi/50:2*pi];
circle_x = 1000+3000*cos(circle_angle);
circle_y = 4750+3000*sin(circle_angle);

switch experiment
    case 1
        distances2all_abs = importdata('dist_1st_tagOnCircleCenter.mat');
    case 2
        distances2all_abs = importdata('dist_2nd_tag_beginFromCenter2_0x2_0x3_back_from_0x1.mat');
            case 3
        distances2all_abs = importdata('dist_3rd_tag_beginFromCenter2_0x2_0x3_back_from_0x1_up_down.mat');
    case 4
        distances2all_abs = importdata('dist_4th_8shape.mat');
    otherwise
        warning('please specify the experiment number #')
end 
    
    positionOfNodes = importdata('nodePos_v2_by_determineNodesPositionBaseOnDistToEachOthers.mat');

% change the unit from m to mm
positionOfNodes = positionOfNodes*1000;
% change the oder to adapt to the nodes oder x2, x3, x1, x5, x6
distances2all_abs = [distances2all_abs(:,2), distances2all_abs(:,3), distances2all_abs(:,1), distances2all_abs(:,5), distances2all_abs(:,6)];
distances2all_abs = distances2all_abs';
%% make cross points of circles with Radius distances2all
% make circles
string =sprintf('experiment %d', experiment);
figure; title(string);hold on; % axis square; axis tight;
plot(positionOfNodes(1,:), positionOfNodes(2,:), 'd');

switch experiment
    case 1
        %--only for experiment 1: plot the circle base on the optical measurements,
        %--base on which the certen of the circle for experi 2&3 is (1000, 4750)
        % optical_measurement = [4250, 5910, 4730, 4670, 5570];
        %     for i = 1 : size(positionOfNodes, 2)
        %         h(i) = plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), ...
        %             optical_measurement(i), 0*pi, 2*pi); % 0, 2*pi); %
        %     end
        % just plot the circle center
        plot(circle_center(1), circle_center(2), '+');
    case 2
        %plot the circle centered in (1000, 4750) which
        %is the traj of experi_2 & traj_projection of experi_3
        plot(circle_center(1), circle_center(2), '+');
        plot(circle_x, circle_y,'-+');
    case 3
        plot(circle_center(1), circle_center(2), '+');
        plot(circle_x, circle_y,'-+');        
    case 4
        plot(circle_center(1), circle_center(2), '+');
    otherwise
        warning('please specify the experiment number #')
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
    title_string = ['j= ', num2str(j)];
    title(title_string);
    axis square; axis tight;
    xlim([-5000 6000]); ylim([-1000 11000]);  
    pause(0.15);
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