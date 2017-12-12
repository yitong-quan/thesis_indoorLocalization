%% x(nodes, tag)
clear;
%% import data
expNum = 2; % <<---- also need to change the one in the 'myfun' below
factor1 = 100;
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

options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);

%% optimization Group0
x_record0 = [];
data_for_opti = group0; %dist_data ; % group_all; % (1:150,:); % group0;% (1:40,:);
myfun0 = @(x)parameterfun(x,data_for_opti); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< choose meas group 
tag_num = size(data_for_opti,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
nodes_num = size(data_for_opti,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x_num = tag_num + nodes_num;
x0 = 10*(rand(2,x_num)-0.5);

resnorm_last = inf;
for ii = 1:7
    [x,resnorm] = lsqnonlin(myfun0,x0,[],[],options);
    resnorm_record = [resnorm_record; resnorm];
    x_record0 = [x_record0;x];
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;
    end    
    x0 = x + 10*(rand(size(x))-0.5);
end
x_opt0 = x_opt;
%%
%% optimization Group1
x_record1 = [];
data_for_opti = [group0; group1]; %dist_data ; % group_all; % (1:150,:); % group0;% (1:40,:);
myfun0 = @(x)parameterfun(x,data_for_opti); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< choose meas group 
tag_num = size(data_for_opti,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
nodes_num = size(data_for_opti,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x_num = tag_num + nodes_num;
new_part_length = size(group1,1);

x0 = [x_opt0, 10*(rand(2,new_part_length)-0.5)];

resnorm_last = inf;
for ii = 1:7
    [x,resnorm] = lsqnonlin(myfun0,x0,[],[],options);
    resnorm_record = [resnorm_record; resnorm];
    x_record1 = [x_record1;x];
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;
    end    
    part0 =  x0(:,1:end-new_part_length) ;
    part1 =  x0(:,end-new_part_length+1:end) ;
    part0 = part0 + 10/factor1*(rand(size(part0))-0.5);
    part1 = part1 + 10*(rand(size(part1))-0.5);
    x0 = [part0, part1];
end
x_opt01 = x_opt;
%%
%% optimization Group2
x_record2 = [];
data_for_opti = [group0; group1;group2]; %dist_data ; % group_all; % (1:150,:); % group0;% (1:40,:);
myfun0 = @(x)parameterfun(x,data_for_opti); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< choose meas group 
tag_num = size(data_for_opti,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
nodes_num = size(data_for_opti,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
x_num = tag_num + nodes_num;
new_part_length = size(group2,1);

x0 = [x_opt01, 10*(rand(2,new_part_length)-0.5)];

resnorm_last = inf;
for ii = 1:15
    [x,resnorm] = lsqnonlin(myfun0,x0,[],[],options);
    resnorm_record = [resnorm_record; resnorm];
    x_record2 = [x_record2;x];
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;
    end    
    part0 =  x0(:,1:end-new_part_length) ;
    part1 =  x0(:,end-new_part_length+1:end) ;
    part0 = part0 + 7/factor1*(rand(size(part0))-0.5);
    part1 = part1 + 7*(rand(size(part1))-0.5);
    x0 = [part0, part1];
end
x_opt012 = x_opt;
%%
% %% optimization Group3 only 2 measurements useless
% x_record3 = [];
% data_for_opti = [group0; group1;group2;group3]; %dist_data ; % group_all; % (1:150,:); % group0;% (1:40,:);
% myfun0 = @(x)parameterfun(x,data_for_opti); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< choose meas group 
% tag_num = size(data_for_opti,1); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< tag_num
% nodes_num = size(data_for_opti,2); %<<<<<<<<<<<<<<<<<<<<<<<<<<<< set nodes_num
% x_num = tag_num + nodes_num;
% new_part_length = size(group3,1);
% 
% x0 = [x_opt012, 10*(rand(2,new_part_length)-0.5)];
% 
% resnorm_last = inf;
% for ii = 1:15
%     [x,resnorm] = lsqnonlin(myfun0,x0,[],[],options);
%     resnorm_record = [resnorm_record; resnorm];
%     x_record3 = [x_record3;x];
%     if resnorm < resnorm_last
%         x_opt = x;
%         resnorm_opt = resnorm;
%         resnorm_last = resnorm_opt;
%     end    
%     part0 =  x0(:,1:end-new_part_length) ;
%     part1 =  x0(:,end-new_part_length+1:end) ;
%     part0 = part0 + 7/factor1*(rand(size(part0))-0.5);
%     part1 = part1 + 7*(rand(size(part1))-0.5);
%     x0 = [part0, part1];
% end
x_opt0123 = x_opt;
%%

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
    plot(x_opt012(1,:), x_opt012(2,:),'r')
    plot(x_opt01(1,:), x_opt01(2,:),'y')
    plot(x_opt0(1,:), x_opt0(2,:),'c')
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
