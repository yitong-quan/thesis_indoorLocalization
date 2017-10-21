clear all
close all
%{
x_num = 55; %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< x_num
nodes_num = 5; %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x0 = 5*rand(2,x_num);
%}

% dist_noisy = importdata('..\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\data_t_dist_1.1-CENTER_t.mat');
dist_noisy = importdata('..\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\data_t_dist_p6_acht_slow_t.mat');
% dist_noisy = importdata('..\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\data_t_dist_p5_acht_t.mat');
dist_noisy = dist_noisy(:,2:end);
% dist_noisy = importdata('..\data-from-experiments\experi_frontDoorTelocate_27.08.2017\movingO_fullData.mat');
 % dist_noisy = importdata('..\data-from-experiments\experi_frontDoorTelocate_27.08.2017\movingO_DataRemoveRowWith0.mat');
% dist_noisy = importdata('..\data-from-experiments\experi_frontDoorTelocate_27.08.2017\3nodes_fixedTag_DataRemoveRowWith0.mat');
dist_noisy = dist_noisy/1000;
%tag_traj = importdata('30points_traj.mat');
tag_num = size(dist_noisy,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
nodes_num = size(dist_noisy,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x_num = tag_num + nodes_num;
x0 = 10*rand(2,x_num);

resnorm_last = inf;
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
for ii = 1:3
    [x,resnorm] = lsqnonlin(@myfun,x0,[],[],options);
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;
    end
    figure;
    axis square; hold on;
    plot(x_opt(1,1:end-nodes_num), x_opt(2,1:end-nodes_num), 'b-*');
    plot(x_opt(1,end-nodes_num+1:end), x_opt(2,end-nodes_num+1:end), 'r-d');
    str = sprintf('x num:%d   resnorm %0.4e ', x_num, resnorm_opt);
    title(str);
    
    x0 = x + 10*rand(size(x));
end
% options = optimoptions(@lsqnonlin,'Algorithm','trust-region-reflective');
% options.Algorithm = 'levenberg-marquardt';

%options = optimoptions('lsqnonlin','Display','iter');
% options.MaxFunctionEvaluations = 6000;
resnorm_opt

%{
for j = 1:nodes_num
    for i = 1:x_num-nodes_num
        opt_x_dist(j,i) = norm(opt_tag(:,i) - opt_node(:,j));
    end
end
%}

opt_tag = x(:,1:end-nodes_num);
opt_node = x(:,end-nodes_num+1:end);
% [x,resnorm] = lsqnonlin(@myfun,x0);
% x_xy = [x(1:length(x)/2); x(length(x)/2+1:end)];
% x_xy
% resnorm
% plot(x_xy(1,:), x_xy(2,:), 'b-*');
% sprev = rng();
% rng(sprev)

% load data
% function [F, true_dist] = myfun(x)  
function [F] = myfun(x)  
%     tag_p_x = linspace(-16,0,17);
%     tag_p = [tag_p_x; zeros(size(tag_p_x))]; 
%     nodes_p = [-7, 3; 0, 0];

%     dist_noisy_insideFunc = importdata('..\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\data_t_dist_1.1-CENTER_t.mat');
    dist_noisy_insideFunc = importdata('..\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\data_t_dist_p6_acht_slow_t.mat');
    % dist_noisy_insideFunc = importdata('..\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\data_t_dist_p5_acht_t.mat');
    dist_noisy_insideFunc = dist_noisy_insideFunc(:,2:end);
    % dist_noisy_insideFunc = importdata('..\data-from-experiments\experi_frontDoorTelocate_27.08.2017\movingO_fullData.mat');
    % dist_noisy_insideFunc = importdata('..\data-from-experiments\experi_frontDoorTelocate_27.08.2017\movingO_DataRemoveRowWith0.mat');
    % dist_noisy_insideFunc = importdata('..\data-from-experiments\experi_frontDoorTelocate_27.08.2017\3nodes_fixedTag_DataRemoveRowWith0.mat');
    dist_noisy_insideFunc = dist_noisy_insideFunc/1000;
    %tag_traj = importdata('30points_traj.mat');
    tag_num = size(dist_noisy_insideFunc,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
    nodes_num = size(dist_noisy_insideFunc,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
    x_num = tag_num + nodes_num; 
%     tag_x = tag_traj(1,:); 
%     tag_y = tag_traj(2,:);
%     tag_p = [tag_x; tag_y]; 
%     nodes_p = [-2, 6, -3, 2, 3; 0, 0, 2, 5, 1];
%     axis square;
%     plot(tag_x, tag_y ,'-*y');
%     hold on;
%     plot(nodes_p(1,:), nodes_p(2,:) ,'-or');

    x_tags = x(:,1:tag_num);
    x_nodes = x(:,end-nodes_num+1:end);
    
    for j = 1:length(x_nodes)
        for i = 1:length(x_tags)
            % true_dist(j,i) = norm(tag_p(:,i) - nodes_p(:,j));
            x_dist(j,i) = norm(x_tags(:,i) - x_nodes(:,j));
        end
    end
    % true_dist = dist_noisy_insideFunc;
    % rng default;
    % true_dist = true_dist + 1.5*( rand(size(true_dist)) - 0.5);
    % x_coordinate = [x(1:length(x)/2); x(length(x)/2+1:end)];  
    F_matrix = x_dist - dist_noisy_insideFunc';
    F = [];
    for i = 1:nodes_num
        F = [F, F_matrix(i,:)];
    end
    %replace NaN with 0 in the F
    F(isnan(F)) = 0;
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

