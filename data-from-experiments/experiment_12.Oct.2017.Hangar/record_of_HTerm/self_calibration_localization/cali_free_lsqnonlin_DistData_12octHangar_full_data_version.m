%% x(tag, nodes)
clear;
close all;

%% import data
expNum = 6;
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
%%
tag_num = size(dist_data,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
nodes_num = size(dist_data,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x_num = tag_num + nodes_num;

x0 = 1*rand(2,x_num);
%% optimization
x_out = opti_fun_caller(x0, @myfun);

%% optimization for group0
resnorm_last = inf;
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
for ii = 1:3
    [x,resnorm] = lsqnonlin(@myfun0,x0,[],[],options);
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;
    end    
    x0 = x + 10*rand(size(x));
end

resnorm_opt
opt_tag = x_opt(:,1:end-nodes_num);
opt_node = x_opt(:,end-nodes_num+1:end);
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
    plot(x_opt(1,1:end-nodes_num), x_opt(2,1:end-nodes_num), 'b-*');
    plot(x_opt(1,end-nodes_num+1:end), x_opt(2,end-nodes_num+1:end), 'r-d');
    str = sprintf('experi: %d;   x num:%d;   resnorm %0.4e ', expNum, x_num, resnorm_opt);
    title(str);
    legend('Tag traj', 'Node');
    daspect([10,10,10]);


%%
function [x_opt] = opti_fun_caller(x_init, fcnHandle)
resnorm_last = inf;
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
for ii = 1:3
    [x,resnorm] = lsqnonlin(fcnHandle,x_init,[],[],options);
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;
    end
    x_init = x + 10*rand(size(x));
end
end
%% myfunction for opti
% load data
% function [F, true_dist] = myfun(x)  
function [F] = myfun(x)  
%% import data
expNum = 3;
switch expNum
    case 1
        dat_t_dist_inside_func = importdata('..\data_t_dist_p1_circle_t.mat');
    case 3
        dat_t_dist_inside_func = importdata('..\data_t_dist_p3_circle_t.mat');
    case 4
        dat_t_dist_inside_func = importdata('..\data_t_dist_p4_circle_t.mat');
    case 5
        dat_t_dist_inside_func = importdata('..\data_t_dist_p5_acht_t.mat');
    case 6
        dat_t_dist_inside_func = importdata('..\data_t_dist_p6_acht_slow_t.mat');
    otherwise
        error('--------- please specify experimen number: expNum');
end

    dist_data_inside_func = dat_t_dist_inside_func(:,2:end);  
    % dist_data_inside_func = dat_t_dist_inside_func(50:200,2:end);    

    dist_data_inside_func = dist_data_inside_func/1000; % unit from mm to m
    tag_num = size(dist_data_inside_func,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
    nodes_num = size(dist_data_inside_func,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
    x_num = tag_num + nodes_num;

    x_tags = x(:,1:tag_num);
    x_nodes = x(:,end-nodes_num+1:end);
    
    for j = 1:length(x_nodes)
        for i = 1:length(x_tags)
            % true_dist(j,i) = norm(tag_p(:,i) - nodes_p(:,j));
            x_dist(i,j) = norm(x_tags(:,i) - x_nodes(:,j));
        end
    end
%{
% F = zeros(size(dist_data_inside_func, 1), 1);
%  for i = 1:size(dist_data_inside_func, 1)
%     dist_this_tag_point = zeros(size(dist_data_inside_func, 2), 1);
%     for j = 1:size(dist_data_inside_func, 2)
%         if ~ isnan(dist_data_inside_func(i,j))
%             dist_this_tag_point(j) = x_dist(i,j) - dist_data_inside_func(i,j);
%         end
%     end
%     F(i) = sum(abs(dist_this_tag_point)); 
%  end    
%}
  dist_this_tag_point = zeros(size(dist_data_inside_func));
  for i = 1:size(dist_data_inside_func, 1)
    for j = 1:size(dist_data_inside_func, 2)
        % isnan(dist_data_inside_func(i,j))
        if  ~isnan(dist_data_inside_func(i,j))
            dist_this_tag_point(i,j) = x_dist(i,j) - dist_data_inside_func(i,j);
            % disp(dist_this_tag_point(i,j))
        end
    end
  end
    F = sum(abs(dist_this_tag_point), 2); 

end


