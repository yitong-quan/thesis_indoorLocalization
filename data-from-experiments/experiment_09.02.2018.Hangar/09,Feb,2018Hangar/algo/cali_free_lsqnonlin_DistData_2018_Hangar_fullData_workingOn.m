%% x(nodes, tag)
%clear;
%% import data from different trajectories
expNum = 1; % <<---- note: also need to change the one in the 'myfun' below
path = '..\';

% switch according to different trajectories
switch expNum
    case 1
        dat_t_dist = importdata([path, 'data_t_dist_data_all.mat']);
    case 2
        dat_t_dist = importdata([path, 'data_t_dist_2_sq_t.mat']);
    case 3
        dat_t_dist = importdata([path, 'data_t_dist_3_sq_t.mat']);
        % dat_t_dist = dat_t_dist(48:133,:);
    otherwise
        error('--------- please specify experimen number: expNum');
end

dist_data = dat_t_dist(:,2:end); % read distance data
dist_data = dist_data/1000; % unit from mm to m

% sort data according to #nan, and sepereate into groups 012345, each presents data set contains only 5, 4, 3, 2, 1 or 0 valid distance data
[numNanMeas,idx]=sort(sum(isnan(dist_data),2));
sorted_dist_data=dist_data(idx,:);

% put data set into different group
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
% data_for_opti: here different combination can be chosen for optimoptions
data_for_opti = dist_data; % index_% dist_data ; % group_all; % (1:150,:); % group0
myfun0 = @(x)parameterfun(x,data_for_opti); % pass parameters using anonymous functions
tag_num = size(data_for_opti,1); % number of tags positions
nodes_num = size(data_for_opti,2); % number of nodes in use
x_num = tag_num + nodes_num; % number of total unkown variables
x0 = 10*(rand(2,x_num)-0.5); % random guess of initial value for these unkown variables
resnorm_last = inf;
for ii = 1:1 % multiple times for optimations and choose one the best results
    [x,resnorm] = lsqnonlin(myfun0,x0,[],[],options); % outpu the optimized positions and the squared 2-norm of the residual  
    resnorm_record = [resnorm_record; resnorm];
    x_record = [x_record;x];
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;
    end    
    x0 = x + 7*(rand(size(x))-0.5); % set next random guess in the neighbour of the current best guess
end
resnorm_opt
opt_tag = x_opt(:,nodes_num+1:end); % best guess of tags trajectory
opt_node = x_opt(:,1:nodes_num); % best guess of nodes' positions

%% RRT: reflection(mirror)-rotation-translation (calculated using only node position)
% transform the coordinate of the best guesses oboved into the one used in 'KF_using_HTerm_data.m'
resnorm_rrt_last = inf;
RRT0 = 10*(rand(1,4)-0.5); % RRT0 = [theta, t1, t2, reflectionAboutXaxis(1or0)]
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
resnorm_last = inf;

% import the positions of nodes determined by 'determineNodesPositionBaseOnDistToEachOthers.m' 
node_po_by_laser = importdata('output\3_RRT\3_opt_node_after_RRT_.mat');
% node_po_by_laser = importdata('..\..\output_algo\nodesPositionLaserOptimal\nodePo.mat'); % 2*5
myfun1 = @(x)parameterfun1(x,node_po_by_laser, opt_node); % pass parameters using anonymous functions
for kk = 1:1 % multiple times for optimations and choose one the best results
    [rrt,resnorm_rrt] = lsqnonlin(myfun1,RRT0,[],[],options);
    if resnorm_rrt < resnorm_rrt_last
        rrt_opt = rrt;
        resnorm_rrt_opt = resnorm_rrt;
        resnorm_rrt_last = resnorm_rrt_opt;        
    end

%% set next starting point   
    RRT0 = rrt + 100*(rand(size(rrt))-0.5); % set next random guess in the neighbour of the current best guess
    RRT0(end) = rand(1)-0.5; % for the reflection flag, randly evenly distr around + and -
end

rrt_opt = ones(1,4);
        if rrt_opt(4) > 0
            reflection_x = 1;
        else
            reflection_x = -1;
        end
Refle_martix = [1, 0; 0, reflection_x];
Rota_matrix =  [cos(rrt_opt(1)), -sin(rrt_opt(1)); sin(rrt_opt(1)), cos(rrt_opt(1))];
Transl_matrix = rrt_opt(2:3)';
% transform the positions into the one used in 'KF_using_HTerm_data.m'
x_opt_after_RRT = Rota_matrix * Refle_martix* x_opt + Transl_matrix; 
opt_tag_after_RRT = x_opt_after_RRT(:,nodes_num+1:end);
opt_node_after_RRT = x_opt_after_RRT(:,1:nodes_num);
%% plot the best result
    h1 = figure;
    hold on;
    plot(opt_tag_after_RRT(1,:), opt_tag_after_RRT(2,:), 'b-x');
    plot(opt_node_after_RRT(1,:), opt_node_after_RRT(2,:), 'kd');
    % plot(node_po_by_laser(1,:), node_po_by_laser(2,:), 'ro'); % only for visual comparison
    str = sprintf('experi: %d;   x num:%d;   resnorm %0.4e ', expNum, x_num, resnorm_opt);
    title(str);
    xlabel('(m)');
    ylabel('(m)');
    legend('Tag estimated', 'Node estimated', 'Node true');
    set(gca,'fontsize',12)
    
    daspect([10,10,10]);
    
    h23= figure;
    plot(resnorm_record);
    title('resnorm record')

%% optimization1: cost funstion is sum of differents of coresponding elements in measured distances and the ones were formed by the solution
% A is the matrix of true distances marix
function F = parameterfun(x, A)
tag_num = size(A,1); % number of tags positions
nodes_num = size(A,2); % number of nodes in use
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
    %replace NaN with [] in the F (PS: replace with 0 has been tried, not working)
    F(isnan(F)) = [];
end

%% optimization2: cost funstion is sum of differents of coresponding points in positions determined by self-calibration and 'KF_using_HTerm_data.m'
function F = parameterfun1(x,B, C) % B is the node positions determined by laser: 2*5; C is the nodes position by the self-calibration: 2*5
data_self_calib_node = C;
node_po_hand_meas = B;
M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];  % x(1):theta, x(2):t1, x(3):t2.
if x(4) > 0 % reflectionAbout_x_Axis(1or0)
    reflection_x = 1;
else
    reflection_x = -1;
end    
Reflection_x_matrix = [1, 0; 0, reflection_x];
afterRT = M(:,[1,2]) * Reflection_x_matrix * data_self_calib_node + M(:,end); % afterRRT = Rota * Refl * nodes_optimal + t0;
dif = afterRT - node_po_hand_meas;
F = zeros(length(dif));
for i = 1:length(F)
    F(i) = norm(dif(:,i));
end
end

