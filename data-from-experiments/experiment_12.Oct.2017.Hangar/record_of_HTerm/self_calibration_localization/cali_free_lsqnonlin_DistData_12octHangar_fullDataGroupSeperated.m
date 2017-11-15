%% x(nodes, tag)

clear;
close all;

%% import data
expNum = 3; % <<---- also need to change the one in the 'myfun' below
switch expNum
    case 1
        dat_t_dist = importdata('..\data_t_dist_p1_circle_t.mat');
    case 3
        dat_t_dist = importdata('..\data_t_dist_p3_circle_t.mat');
    case 4
        dat_t_dist = importdata('..\data_t_dist_p4_circle_t.mat');
    case 5
        dat_t_dist = importdata('..\data_t_dist_p5_acht_t.mat');
    case 6
        dat_t_dist = importdata('..\data_t_dist_p6_acht_slow_t.mat');
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
tag_num = size(group0,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
nodes_num = size(group0,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x_num = tag_num + nodes_num;
x0 = 10*rand(2,x_num);
myfun0 = @(x)parameterfun(x,group0);
%% optimization
resnorm_last = inf;
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
for ii = 1:1
    [x,resnorm] = lsqnonlin(myfun0,x0,[],[],options);
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;
    end    
    x0 = x + 10*rand(size(x));
end
resnorm_opt
opt_tag = x_opt(:,nodes_num+1:end);
opt_node = x_opt(:,1:nodes_num);
%{
for j = 1:nodes_num
    for i = 1:x_num-nodes_num
        opt_x_dist(j,i) = norm(opt_tag(:,i) - opt_node(:,j));
    end
end
%}

%% plot the best result
    figure;
    hold on;
    plot(opt_tag(1,:), opt_tag(2,:), 'b-*');
    plot(opt_node(1,:), opt_node(2,:), 'r-d');
    str = sprintf('experi: %d;   x num:%d;   resnorm %0.4e ', expNum, x_num, resnorm_opt);
    title(str);
    legend('Tag traj', 'Node');
    daspect([10,10,10]);

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
% F = sum(abs(dist_this_tag_point), 2);
    F_matrix = x_dist - A;
    F = [];
    for i = 1:nodes_num
        F = [F, F_matrix(i,:)];
    end
    %replace NaN with 0 in the F
    F(isnan(F)) = 0;
end
