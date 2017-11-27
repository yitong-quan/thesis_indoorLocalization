
expNum = 1;

path = '..\data\';
switch expNum
    case 1
        dat_t_dist = importdata([path, 'data_t_dist_all_t.mat']);
    otherwise
        error('--------- please specify experimen number: expNum');
end
time = dat_t_dist(:,1);
time_Diff = diff(time);
% time_Diff(time_Diff<0)=[];
time_Diff(time_Diff>15)=[];
figure;
histogram(time_Diff);
%     title('hist time interval');
xlabel('time interval (s)');

dist_data = dat_t_dist(:,2:end); %dist_data = dat_t_dist(50:200,2:end);
dist_data = dist_data/1000; % unit from mm to m

    numNan = sum(isnan(dist_data),2);
    numMeas_each_set = size(dist_data,2) - numNan;
    figure
% subplot(2,2,i)
    histogram(numMeas_each_set);
    xlim([-1 6]);
    title('hist useful numMeas');
    xlabel('num of useful measurements each set');
%     i=i+1;
    num0meas = sum(numMeas_each_set==0);
    num1meas = sum(numMeas_each_set==1);
    num2meas = sum(numMeas_each_set==2);
    num3meas = sum(numMeas_each_set==3);
    num4meas = sum(numMeas_each_set==4);
    num5meas = sum(numMeas_each_set==5);
    num_x_meas = [num0meas num1meas num2meas num3meas num4meas num5meas];
    procentage = num_x_meas/ size(dist_data,1);
    NUM_X_MEAS = [];
        NUM_MEAS = [];
            PROCENT = [];
    NUM_X_MEAS =[NUM_X_MEAS; num_x_meas]; %colunm:exper#; row:#012345meas
    NUM_MEAS = [NUM_MEAS; size(dist_data,1)];%colunm:exper#
    PROCENT = [PROCENT; procentage]; %colunm:exper#; row:#012345meas