%% x(tag, nodes)
clear;
close all;

%% import data
expNum = 3;
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
%{
% %% remove dat_t_dist with value NaN
% dat_t_dist_trimed = double(dat_t_dist);
% for i = size(dat_t_dist_trimed,1):-1:1
%     for j = 1: size(dat_t_dist_trimed,2)
%         if isnan(dat_t_dist_trimed(i,j))
%             dat_t_dist_trimed(i,:) = [];
%             break;
%         end
%     end
% end
% dist_data = dat_t_dist_trimed(:,2:end);
%}
dist_data = dat_t_dist(:,2:end);
dist_data = dist_data/1000; % unit from mm to m
tag_num = size(dist_data,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
nodes_num = size(dist_data,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x_num = tag_num + nodes_num;
x0 = 1*rand(2,x_num);
%% optimization
resnorm_last = inf;
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
for ii = 1:3
    [x,resnorm] = lsqnonlin(@myfun,x0,[],[],options);
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;
    end    
    x0 = x + 10*(rand(size(x))-0.5);
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

% [x,resnorm] = lsqnonlin(@myfun,x0);
% x_xy = [x(1:length(x)/2); x(length(x)/2+1:end)];
% x_xy
% resnorm
% plot(x_xy(1,:), x_xy(2,:), 'b-*');
% sprev = rng();
% rng(sprev)

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
%{
%% remove dat_t_dist with value NaN
%     dat_t_dist_trimed_inside_func = double(dat_t_dist_inside_func);
%     for i = size(dat_t_dist_trimed_inside_func,1):-1:1
%         for j = 1: size(dat_t_dist_trimed_inside_func,2)
%             if isnan(dat_t_dist_trimed_inside_func(i,j))
%                 dat_t_dist_trimed_inside_func(i,:) = [];
%                 break;
%             end
%         end
%     end
%     dist_data_inside_func = dat_t_dist_trimed_inside_func(:,2:end);
%}     
    dist_data_inside_func = dat_t_dist_inside_func(:,2:end);   

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

%   dist_this_tag_point = zeros(size(dist_data_inside_func));
%   for i = 1:size(dist_data_inside_func, 1)
%     for j = 1:size(dist_data_inside_func, 2)
%         % isnan(dist_data_inside_func(i,j))
%         if  ~isnan(dist_data_inside_func(i,j))
%             dist_this_tag_point(i,j) = x_dist(i,j) - dist_data_inside_func(i,j);
%             % disp(dist_this_tag_point(i,j))
%         end
%     end
%   end
%     F = sum(abs(dist_this_tag_point), 2); 

dist_this_tag_point = x_dist - dist_data_inside_func;
dist_this_tag_point(isnan(dist_this_tag_point)) = 0;
F = [];
    for i = 1:nodes_num
        F = [F; dist_this_tag_point(:,i)];
    end



end

%{
function [F, true_dist] = myfun(x)  
%     tag_p_x = linspace(-16,0,17);
%     tag_p = [tag_p_x; zeros(size(tag_p_x))]; 
%     nodes_p = [-7, 3; 0, 0];
    x_num = 55; %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< x_num
    nodes_num = 5; %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
    tag_num = x_num - nodes_num;
    t = linspace(0,2*pi,tag_num);
    tag_x = 5*cos(t); % 5*t-20;
    tag_y = 10*sin(t)+12;
    tag_p = [tag_x; tag_y]; 
    nodes_p = [-20, 60, -30, 20, 3; 0, 0, 25, 25, 15.9];
    plot(tag_x, tag_y ,'-*y');
    axis square;
    hold on;
    plot(nodes_p(1,:), nodes_p(2,:) ,'-or');

    x_tags = x(:,1:end-nodes_num);
    x_nodes = x(:,end-nodes_num+1:end);
    
    for j = 1:length(nodes_p)
        for i = 1:length(tag_p)
            true_dist(j,i) = norm(tag_p(:,i) - nodes_p(:,j));
            x_dist(j,i) = norm(x_tags(:,i) - x_nodes(:,j));
        end
    end
    rng default;
    true_dist = true_dist + 3*( rand(size(true_dist)) - 0.5);
    % x_coordinate = [x(1:length(x)/2); x(length(x)/2+1:end)];  
    F_matrix = x_dist - true_dist;
    F = [];
    for i = 1:nodes_num
        F = [F, F_matrix(i,:)];
    end
end
%}
