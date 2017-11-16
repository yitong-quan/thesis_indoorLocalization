%% using only the   ---nodes positions---   for self-calibration
% nodes position estimated by the file 'cali_free_lsqnonlin_importDistData_12octHangar.m'
% compare with the hand measured value
% reference coordinate hand measured value

% file locates in 'D:\Yitong\GitHub\thesis_indoorLocalization\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\self_calibration_localization\...'
clear
%% process data
expNum = 3;

% str_self_calib_node = ['results/remove dataSet with NaN element/exper', num2str(expNum), '_opt_node.mat'];
str_self_calib_node = ['results/full data/exper', num2str(expNum), 'group0_opt_node.mat'];
data_self_calib_node = importdata(str_self_calib_node);
node_po_hand_meas = importdata('../nodePos_by_determineNodesPositionBaseOnDistToEachOthers.mat');

%{
% replace outliar with the closest 'correct' position
% data_US(2:3,1:3) = repmat(data_US(2:3,4),1,3);
% % nearestpoint(shortArray,longArray)
% timeStampNeed_data_US = nearestpoint(data_self_calib(5,:), data_US(4,:));
%{
% check for timeStampNeed_data_US
diff=zeros(1,size(data_self_calib,2));
for i = 1:size(data_self_calib,2)
    diff(i) = data_self_calib(5,i) - data_US(4, timeStampNeed_data_US(i));
end
%}

% data with the row_index of timeStampNeed_data_US
data_US_trimed = data_US(:,timeStampNeed_data_US);
%}

%% (M)calculate the translateion matrix R and T
x0 = 10*(rand(1,4)-0.5); % M = [theta, t1, t2, reflectionAboutXaxis(1or0)]
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
resnorm_last = inf;
for ii = 1:6
    [x,resnorm] = lsqnonlin(@myfun,x0,[],[],options);
    if resnorm < resnorm_last
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;        
    end

%% set next starting point   
    x0 = x + 100*(rand(size(x))-0.5);
    x0(end) = rand(1)-0.5; % for the reflection flag, randly evenly distr around + and -
end

        if x_opt(4) > 0
            reflection_x = 1;
        else
            reflection_x = -1;
        end
Refle_martix = [1, 0; 0, reflection_x];
Rota_matrix =  [cos(x_opt(1)), -sin(x_opt(1)); sin(x_opt(1)), cos(x_opt(1))];
Transl_matrix = x_opt(2:3)';


%% plot ---nodes positions--- of self _calib and hand meas
data_self_calib_node_after_RRT = Rota_matrix * Refle_martix* data_self_calib_node + Transl_matrix;
figure; plot(node_po_hand_meas(1,:), node_po_hand_meas(2,:), 'r-d');
hold on;
plot(data_self_calib_node(1,:), data_self_calib_node(2,:),'g-d'); 
plot(data_self_calib_node_after_RRT(1,:), data_self_calib_node_after_RRT(2,:), 'b-d');

daspect([10,10,10]);

%% import Assist data and rotate in to hand_measurement coordinate and plot
str_US = ['../../record_of_ultra_sound_system/positionBeforeMakedBetterFromJoan/mat_positions', num2str(expNum), '.mat'];
% str_US = ['../../record_of_ultra_sound_system/positionsBetterFromJoan/mat_positions', num2str(expNum), '.mat'];
data_US = importdata(str_US);
data_US = data_US';
data_US_p = data_US(2:3,:);
str_RT2hand_measure_coordinate = ['../forAlignment\beforeBetterFromJoan/forAlignment_RT_ASSIS_2_UWB_exp', num2str(expNum), '.mat'];
M_RT2h_m = importdata(str_RT2hand_measure_coordinate);
data_US_p_RT2h_m = M_RT2h_m(:,[1,2]) *data_US_p + M_RT2h_m(:,end);

plot(data_US_p_RT2h_m(1,:), data_US_p_RT2h_m(2,:),'r-+'); 
legend('node hand meas', 'node self calib', 'node self calib after RRT', 'Assit Traj');
title_stri = ['exp', num2str(expNum), '  reflection_X=', num2str(x_opt(4)), '  theta=', num2str(x_opt(1)), '  t1=', num2str(x_opt(2)), '  t2=', num2str(x_opt(3))];
title(title_stri);


%% optimazation used matrix
% afterRT = R0 * nodes_optimal + t0;
function F = myfun(x)

expNum = 3;

%str_self_calib_node = ['results/remove dataSet with NaN element/exper', num2str(expNum), '_opt_node.mat'];
str_self_calib_node = ['results/full data/exper', num2str(expNum), 'group0_opt_node.mat'];
data_self_calib_node = importdata(str_self_calib_node);
node_po_hand_meas = importdata('../nodePos_by_determineNodesPositionBaseOnDistToEachOthers.mat');
%{
% check for timeStampNeed_data_US
diff=zeros(1,size(data_self_calib,2));
for i = 1:size(data_self_calib,2)
    diff(i) = data_self_calib(5,i) - data_US(4, timeStampNeed_data_US(i));
end
%}

M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];
if x(4) > 0
    reflection_x = 1;
else
    reflection_x = -1;
end    
Reflection_x_matrix = [1, 0; 0, reflection_x];
afterRT = M(:,[1,2]) * Reflection_x_matrix * data_self_calib_node + M(:,end);
dif = afterRT - node_po_hand_meas;
mis_dist = zeros(length(dif));
for i = 1:length(mis_dist)
    mis_dist(i) = norm(dif(:,i));
end
F = mis_dist;
end

