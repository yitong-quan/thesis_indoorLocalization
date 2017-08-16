clear all
close all

x_num = 55; %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< x_num
    nodes_num = 5; %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x0 = 5*rand(2,x_num);
% options = optimoptions(@lsqnonlin,'Algorithm','trust-region-reflective');
% options.Algorithm = 'levenberg-marquardt';
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
%options = optimoptions('lsqnonlin','Display','iter');
% options.MaxFunctionEvaluations = 6000;
[x,resnorm] = lsqnonlin(@myfun,x0,[],[],options);
plot(x(1,1:end-nodes_num), x(2,1:end-nodes_num), 'b-*');
plot(x(1,end-nodes_num+1:end), x(2,end-nodes_num+1:end), 'b-o');
resnorm
opt_tag = x(:,1:end-nodes_num);
opt_node = x(:,end-nodes_num+1:end);
for j = 1:nodes_num
    for i = 1:x_num-nodes_num
        opt_x_dist(j,i) = norm(opt_tag(:,i) - opt_node(:,j));
    end
end
str = sprintf('x num:%d   resnorm %0.4e ', x_num, resnorm);
title(str);
% [x,resnorm] = lsqnonlin(@myfun,x0);
% x_xy = [x(1:length(x)/2); x(length(x)/2+1:end)];
% x_xy
% resnorm
% plot(x_xy(1,:), x_xy(2,:), 'b-*');
sprev = rng();
rng(sprev)


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
    true_dist = true_dist + 1*( rand(size(true_dist)) - 0.5);
    % x_coordinate = [x(1:length(x)/2); x(length(x)/2+1:end)];  
    F_matrix = x_dist - true_dist;
    F = [];
    for i = 1:nodes_num
        F = [F, F_matrix(i,:)];
    end
end
%{
x_addnoise = x + 0.1*rand
(norm(myfun4testing(x_addnoise)))^2 - resnorm
function F = myfun4testing(x)    
    F = [(x(1) - 2)^2;(x(2) - 3)^2;(x(3) - 4)^2;...
        (x(4) - 5)^2;(x(5) - 6)^2;(x(6) - 7)^2;];
end
%}

