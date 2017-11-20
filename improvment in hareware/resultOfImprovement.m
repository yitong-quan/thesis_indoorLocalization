NUM_X_MEAS = [];
NUM_MEAS = [];
PROCENT = [];
beforeOrAfter = 1; % before1 after0
if beforeOrAfter == 1
    A = [2,3,3.1,5];
    path = 'before\';
else 
    A = [1,3,4,56];
    path = '..\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\';
end    
 figure;
 title('hist time interval');
 i = 1;
 
for expNum = A
    if beforeOrAfter == 1
        switch expNum
            case 2
                dat_t_dist = importdata([path, 'data_t_dist_2.txt.mat']);
            case 3
                dat_t_dist = importdata([path, 'data_t_dist_3.txt.mat']);
            case 3.1
                dat_t_dist = importdata([path, 'data_t_dist_3_1.txt.mat']);
            case 5
                dat_t_dist = importdata([path, 'data_t_dist_5.txt.mat']);
            otherwise
                error('--------- please specify experimen number: expNum');
        end
    else
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
    end
    time = dat_t_dist(:,1);
    time_Diff = diff(time);
    time_Diff(time_Diff<0)=[];
    time_Diff(time_Diff>15)=[];
%     figure;
    subplot(2,2,i)
    histogram(time_Diff);
%     title('hist time interval');
    xlabel('time interval (s)');
    i=i+1;
     
    dist_data = dat_t_dist(:,2:end); %dist_data = dat_t_dist(50:200,2:end);
    dist_data = dist_data/1000; % unit from mm to m
%     
%     numNan = sum(isnan(dist_data),2);
%     numMeas_each_set = size(dist_data,2) - numNan;
%     figure
% % subplot(2,2,i)
%     histogram(numMeas_each_set);
%     xlim([-1 6]);
%     title('hist useful numMeas');
%     xlabel('num of useful measurements each set');
% %     i=i+1;
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

% if beforeOrAfter == 1
%     save('before\NUM_X_MEAS.mat', 'NUM_X_MEAS');
%     save('before\NUM_MEAS.mat', 'NUM_MEAS');
%     save('before\PROCENT.mat', 'PROCENT');
% else
%     A = [1,3,4,5,6];
%     save('after\NUM_X_MEAS.mat', 'NUM_X_MEAS');
%     save('after\NUM_MEAS.mat', 'NUM_MEAS');
%     save('after\PROCENT.mat', 'PROCENT');
% end   