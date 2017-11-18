NUM_X_MEAS = [];
NUM_MEAS = [];
PROCENT = [];
for expNum = [1,3,4,5,6];
    path = '..\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\';
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
        otherwise
            error('--------- please specify experimen number: expNum');
    end
    time = dat_t_dist(:,1);
    time_Diff = diff(time);
    figure;
    hist(time_Diff,100);
    title('time Diff');
    dist_data = dat_t_dist(:,2:end); %dist_data = dat_t_dist(50:200,2:end);
    dist_data = dist_data/1000; % unit from mm to m
    
    numNan = sum(isnan(dist_data),2);
    numMeas_each_set = size(dist_data,2) - numNan;
    figure
    hist(numMeas_each_set);
    num0meas = sum(numMeas_each_set==0);
    num1meas = sum(numMeas_each_set==1);
    num2meas = sum(numMeas_each_set==2);
    num3meas = sum(numMeas_each_set==3);
    num4meas = sum(numMeas_each_set==4);
    num5meas = sum(numMeas_each_set==5);
    num_x_meas = [num0meas num1meas num2meas num3meas num4meas num5meas];
    procentage = num_x_meas/ size(dist_data,1);
    NUM_X_MEAS =[NUM_X_MEAS; num_x_meas]; %colunm:exper#; row:#012345meas
    NUM_MEAS = [NUM_MEAS; size(dist_data,1)];%colunm:exper#
    PROCENT = [PROCENT; procentage]; %colunm:exper#; row:#012345meas
end