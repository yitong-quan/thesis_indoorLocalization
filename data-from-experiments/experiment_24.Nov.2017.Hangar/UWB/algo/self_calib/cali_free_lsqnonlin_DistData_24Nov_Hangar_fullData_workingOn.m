%% x(nodes, tag)
clear;
%% import data
expNum = 3; % <<---- also need to change the one in the 'myfun' below
path = '..\..\data\';

switch expNum
    case 1
        dat_t_dist = importdata([path, 'data_t_dist_1circle_t.mat']);
    case 2
        dat_t_dist = importdata([path, 'data_t_dist_2_sq_t.mat']);
    case 3
        dat_t_dist = importdata([path, 'data_t_dist_3_sq_t.mat']);
    otherwise
        error('--------- please specify experimen number: expNum');
end
dist_data = dat_t_dist(:,2:end); %dist_data = dat_t_dist(50:200,2:end);
dist_data = dist_data/1000; % unit from mm to m
%% sort data according to #nan, and sepereate into groups 012345
[numNanMeas,idx]=sort(sum(isnan(dist_data),2));
sorted_dist_data=dist_data(idx,:);

group0 = []; group1 = []; group2 = []; group3 = []; group4 = []; group5 = []; 
for ij = 1:length(numNanMeas)
    if numNanMeas(ij) == 0
        group0 = [group0; sorted_dist_data(ij,:)];
    end
    if numNanMeas(ij) == 1
        group1 = [group1; sorted_dist_data(ij,:)];
    end
    if numNanMeas(ij) == 2
        group2 = [group2; sorted_dist_data(ij,:)];
    end
    if numNanMeas(ij) == 3
        group3 = [group3; sorted_dist_data(ij,:)];
    end
    if numNanMeas(ij) == 4
        group4 = [group4; sorted_dist_data(ij,:)];
    end
    if numNanMeas(ij) == 5
        group5 = [group5; sorted_dist_data(ij,:)];
    end
end    

resnorm_record = [];
x_record = [];
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);

%% optimization

group_all = sorted_dist_data;
index_ = [1,20, 40,60,80,100,120];
data_for_opti = dist_data; % group0(1:4,:); %index_% dist_data ; % group_all; % (1:150,:); % group0;% (1:40,:);<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
myfun0 = @(x)parameterfun(x,data_for_opti); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< choose meas group 
tag_num = size(data_for_opti,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
nodes_num = size(data_for_opti,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x_num = tag_num + nodes_num;
x0 = 10*(rand(2,x_num)-0.5);
resnorm_last = inf;
for ii = 1:7
    [x,resnorm] = lsqnonlin(myfun0,x0,[],[],options);
    resnorm_record = [resnorm_record; resnorm];
    x_record = [x_record;x];
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;
    end    
    x0 = x + 7*(rand(size(x))-0.5);
end
resnorm_opt
opt_tag = x_opt(:,nodes_num+1:end);
opt_node = x_opt(:,1:nodes_num);
%% RRT
resnorm_rrt_last = inf;
RRT0 = 10*(rand(1,4)-0.5); % M = [theta, t1, t2, reflectionAboutXaxis(1or0)]
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
resnorm_last = inf;
node_po_by_laser = importdata('..\..\output_algo\nodesPositionLaserOptimal\nodePo.mat'); % 2*5
myfun1 = @(x)parameterfun1(x,node_po_by_laser, opt_node);
for kk = 1:6
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
x_opt_after_RRT = Rota_matrix * Refle_martix* x_opt + Transl_matrix;
opt_tag_after_RRT = x_opt_after_RRT(:,nodes_num+1:end);
opt_node_after_RRT = x_opt_after_RRT(:,1:nodes_num);
%% plot the best result
    figure;
    hold on;
    % plot(opt_tag(1,:), opt_tag(2,:), 'k-*');
    % plot(opt_node(1,:), opt_node(2,:), 'gd');
    plot(opt_tag_after_RRT(1,:), opt_tag_after_RRT(2,:), 'b-x');
    plot(opt_node_after_RRT(1,:), opt_node_after_RRT(2,:), 'kd');
    plot(node_po_by_laser(1,:), node_po_by_laser(2,:), 'ro');
    str = sprintf('experi: %d;   x num:%d;   resnorm %0.4e ', expNum, x_num, resnorm_opt);
    title(str);
    %  xlabel('(m)','FontSize',24,'FontWeight','bold');
    xlabel('(m)');
    ylabel('(m)');
    legend('Tag estimated', 'Node estimated', 'Node true');
    set(gca,'fontsize',12)
    
    daspect([10,10,10]);
    
    figure;
    plot(resnorm_record);
    title('resnorm record')

%% parafun including meas data
% A is the matrix of true meas marix
function F = parameterfun(x, A)
tag_num = size(A,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
nodes_num = size(A,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x_num = tag_num + nodes_num;
if x_num ~= length(x)
    error('size(x_num) isnot eual to size(x)');
end
x_nodes = x(:,1:nodes_num);
x_tags = x(:,nodes_num+1:end);
for i = 1:length(x_tags)
    for j = 1:length(x_nodes)
        % true_dist(j,i) = norm(tag_p(:,i) - nodes_p(:,j));
        x_dist(i,j) = norm(x_tags(:,i) - x_nodes(:,j));
    end
end
%{
% dist_this_tag_point = zeros(size(A));
% for i = 1:size(A, 1) % #tag
%     for j = 1:size(A, 2) % #nodes
%         % isnan(dist_data_inside_func(i,j))
%         if  ~isnan(A(i,j))
%             dist_this_tag_point(i,j) = x_dist(i,j) - A(i,j);
%             % disp(dist_this_tag_point(i,j))
%         end
%     end
% end
% %F = sum(abs(dist_this_tag_point), 2);
% F = sum((dist_this_tag_point).^2, 2);
%}
    F_matrix = x_dist - A;
    F_matrix = F_matrix';
    F = [];
    for i = 1:nodes_num
        F = [F, F_matrix(i,:)];
    end
    %replace NaN with [] in the F (with 0 has been tried, not working)
    F(isnan(F)) = [];
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

