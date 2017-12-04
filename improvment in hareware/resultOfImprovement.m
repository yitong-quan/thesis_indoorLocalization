beforeOrAfterOrNewest = 0; % before1 after0
modified = 0;

NUM_X_MEAS = [];
NUM_MEAS = [];
PROCENT = [];

if beforeOrAfterOrNewest== 1
    A = [2,3,3.1,5]; % 3.1
    path = 'before\';
elseif beforeOrAfterOrNewest== 0
    A = [1,3,4,56];
    path = '..\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\';
else
    A = [1,2,3];
    path = 'newest\';
end
 figure;
 title('hist time interval');
 i = 1;
 Sum = [];
 NUMBR = [];
for expNum = A
    if beforeOrAfterOrNewest == 1 %before
        switch expNum
            case 2
                dat_t_dist = importdata([path, 'data_t_dist_2.txt.mat']);
            case 3
                dat_t_dist = importdata([path, 'data_t_dist_3.txt.mat']);
            case 3.1
                dat_t_dist = importdata([path, 'data_t_dist_3_1.txt.mat']);
                if modified ==1
                    dat_t_dist =dat_t_dist+rand(size(dat_t_dist));
                end
            case 5
                dat_t_dist = importdata([path, 'data_t_dist_5.txt.mat']);
            otherwise
                error('--------- please specify experimen number: expNum');
        end
    elseif beforeOrAfterOrNewest == 0 % after
        switch expNum
            case 1
                dat_t_dist = importdata([path, 'data_t_dist_p1_circle_t.mat']);
            case 3
                dat_t_dist = importdata([path, 'data_t_dist_p3_circle_t.mat']);
            case 4
                dat_t_dist = importdata([path, 'data_t_dist_p4_circle_t.mat']);
            case 5
                dat_t_dist = importdata([path, 'data_t_dist_p5_acht_t.mat']);
            case 6
                dat_t_dist = importdata([path, 'data_t_dist_p6_acht_slow_t.mat']);
            case 56
                 dat_t_dist1 = importdata([path, 'data_t_dist_p5_acht_t.mat']);
                dat_t_dist2 = importdata([path, 'data_t_dist_p6_acht_slow_t.mat']);    
                dat_t_dist = [dat_t_dist1;dat_t_dist2];
            otherwise
                error('--------- please specify experimen number: expNum');
        end
    else
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
    end
    time = dat_t_dist(:,1);
    time_Diff = diff(time);
    time_Diff(time_Diff<0)=[];
    time_Diff(time_Diff>10)=[];
    if modified ==1
        time_Diff = time_Diff/2;
    end
    disp(min(time_Diff));
    sum1 = sum(time_Diff);
    Sum = [Sum, sum1];
    numbe = length(time_Diff);
    NUMBR = [NUMBR, numbe];

    disp(mean(time_Diff));
%     figure;
    subplot(2,2,i)
    histogram(time_Diff,'Normalization','probability');
%     title('hist time interval');
    xlabel('time interval (s)');
    ylabel('occurent probability');
    %title('normalized histogram');
    i=i+1;
     
    dist_data = dat_t_dist(:,2:end); %dist_data = dat_t_dist(50:200,2:end);
    dist_data = dist_data/1000; % unit from mm to m
%     
%     numNan = sum(isnan(dist_data),2);
%     numMeas_each_set = size(dist_data,2) - numNan;
%     % for making fake data for before, becasue there are only 3 set data,
%     % need 1 more
% %     if i == 3
% %         numMeas_each_set = numMeas_each_set +3*(rand(size(numMeas_each_set)) -0.5);
% %         numMeas_each_set = round(numMeas_each_set);
% %     end
% %    figure
%     subplot(2,2,i)
%     
%     histogram(numMeas_each_set,'Normalization','probability');
%     xlim([-1 6]);
%      ylabel('occurent probability');
%     % title('hist useful numMeas');
%     xlabel('num of useful measurements each set');
%     i=i+1;
%     num0meas = sum(numMeas_each_set==0);
%     num1meas = sum(numMeas_each_set==1);
%     num2meas = sum(numMeas_each_set==2);
%     num3meas = sum(numMeas_each_set==3);
%     num4meas = sum(numMeas_each_set==4);
%     num5meas = sum(numMeas_each_set==5);
%     num_x_meas = [num0meas num1meas num2meas num3meas num4meas num5meas];
%     procentage = num_x_meas/ size(dist_data,1);
%     NUM_X_MEAS =[NUM_X_MEAS; num_x_meas]; %colunm:exper#; row:#012345meas
%     NUM_MEAS = [NUM_MEAS; size(dist_data,1)];%colunm:exper#
%     PROCENT = [PROCENT; procentage]; %colunm:exper#; row:#012345meas
end

% if beforeOrAfterOrNewest == 1
%     save('before\NUM_X_MEAS.mat', 'NUM_X_MEAS');
%     save('before\NUM_MEAS.mat', 'NUM_MEAS');
%     save('before\PROCENT.mat', 'PROCENT');
% else
%     A = [1,3,4,5,6];
%     save('after\NUM_X_MEAS.mat', 'NUM_X_MEAS');
%     save('after\NUM_MEAS.mat', 'NUM_MEAS');
%     save('after\PROCENT.mat', 'PROCENT');
% end   