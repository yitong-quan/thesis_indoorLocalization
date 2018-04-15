data_read = importdata('output\calibration_free_workspace_3.mat'); 
opt_node_ = data_read.opt_node_after_RRT;
%% plot nodes
figure; plot(opt_node_(1,:),opt_node_(2,:)); hold on
plot(opt_node_(1,1),opt_node_(2,1),'o');
plot(opt_node_(1,2),opt_node_(2,2),'+');
plot(opt_node_(1,3),opt_node_(2,3),'^');
plot(opt_node_(1,4),opt_node_(2,4),'s');
plot(opt_node_(1,5),opt_node_(2,5),'p');
daspect([10,10,10]);
%%%%%%%% 3.3, 0.1
%% mark distances
dist_nodes12 = norm(opt_node_(:,1)-opt_node_(:,2));
text(0.5*(opt_node_(1,1)+opt_node_(1,2)),...
    0.5*(opt_node_(2,1)+opt_node_(2,2)),num2str(dist_nodes12));
dist_nodes23 = norm(opt_node_(:,2)-opt_node_(:,3));
text(0.5*(opt_node_(1,2)+opt_node_(1,3)),...
    0.5*(opt_node_(2,2)+opt_node_(2,3)),num2str(dist_nodes23));
dist_nodes34 = norm(opt_node_(:,3)-opt_node_(:,4));
text(0.5*(opt_node_(1,3)+opt_node_(1,4)),...
    0.5*(opt_node_(2,3)+opt_node_(2,4)),num2str(dist_nodes34));
dist_nodes45 = norm(opt_node_(:,4)-opt_node_(:,5));
text(0.5*(opt_node_(1,4)+opt_node_(1,5)),...
    0.5*(opt_node_(2,4)+opt_node_(2,5)),num2str(dist_nodes45));
dist_nodes51 = norm(opt_node_(:,5)-opt_node_(:,1));
text(0.5*(opt_node_(1,5)+opt_node_(1,1)),...
    0.5*(opt_node_(2,5)+opt_node_(2,1)),num2str(dist_nodes51));
%% RRT
resnorm_rrt_last = inf;
RRT0 = 10*(rand(1,4)-0.5); % M = [theta, t1, t2, reflectionAboutXaxis(1or0)]
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
resnorm_last = inf;
node_po_by_laser = [dist_nodes23,0; 0,0]; % 2*2
myfun1 = @(x)parameterfun1(x,node_po_by_laser, opt_node_(:,2:3));
for kk = 1:60
    [rrt,resnorm_rrt] = lsqnonlin(myfun1,RRT0,[],[],options);
    if resnorm_rrt < resnorm_rrt_last
        rrt_opt = rrt;
        resnorm_rrt_opt = resnorm_rrt;
        resnorm_rrt_last = resnorm_rrt_opt;        
    end
%% set next starting point   
    RRT0 = rrt + 100*(rand(size(rrt))-0.5);
    RRT0(end) = rand(1)-0.5; % for the reflection flag, randly evenly distr around + and -
end
        if rrt_opt(4) > 0
            reflection_x = 1;
        else
            reflection_x = -1;
        end
Refle_martix = [1, 0; 0, reflection_x];
Rota_matrix =  [cos(rrt_opt(1)), -sin(rrt_opt(1)); sin(rrt_opt(1)), cos(rrt_opt(1))];
Transl_matrix = rrt_opt(2:3)';
x_opt_after_RRT_ = Rota_matrix * Refle_martix* x_opt_after_RRT + Transl_matrix;
opt_tag_after_RRT_ = x_opt_after_RRT_(:,5+1:end);
opt_node_after_RRT_ = x_opt_after_RRT_(:,1:5);

plot(opt_node_after_RRT_(1,:),opt_node_after_RRT_(2,:));
plot(opt_node_after_RRT_(1,1),opt_node_after_RRT_(2,1),'o');
%% plot circles 
for j = 1:size(dat_t_dist, 1)
    % h_1 = plot(traj(1,j), traj(2,j), 'ro'); %hold on;
        for i = 1 : 5
            h(i) = plotCircle(opt_node_(1, i), opt_node_(2, i), ...
                dat_t_dist(j,i+1)/1000, 0*pi, 2*pi); % 0, 2*pi); %
        end
    axis square; axis tight;
    xlim([-5 5]); ylim([-6 6]);  
    daspect([10,10,10]);
    string =sprintf('j = %d', j);
    title(string);
    pause(0.05);

    delete(h); 
end



%% optimazation used matrix
% afterRRT = Rota*Refl nodes_optimal + t0;
function F = parameterfun1(x,B, C) % B is the node_laser matrix: 2*5; C is the node_self_calib matrix: 2*5
data_self_calib_node = C;
node_po_hand_meas = B;
M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];
if x(4) > 0
    reflection_x = 1;
else
    reflection_x = -1;
end    
Reflection_x_matrix = [1, 0; 0, reflection_x];
afterRT = M(:,[1,2]) * Reflection_x_matrix * data_self_calib_node + M(:,end);
dif = afterRT - node_po_hand_meas;
F = zeros(length(dif));
for i = 1:length(F)
    F(i) = norm(dif(:,i));
end
end


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