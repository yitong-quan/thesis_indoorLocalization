%data contain: [timeref	est_sendtimepos_x	pos_y	vel_x	vel_y sigma	valid	senderGuid]
function read_and_generate_TXYt_matrix_from_ultra_sound_system(i)
%M = dlmread(filename,delimiter,R1,C1) starts reading at row offset R1 and column offset C1. For example, the offsets R1=0, C1=0 specify the first value in the file.

%folder_name = 'positionBeforeMakedBetterFromJoan';
folder_name = 'positionsBetterFromJoan';
file_str_pre = 'positions';
file_name = [folder_name, '/', file_str_pre, num2str(i), '.log'];

data  = dlmread(file_name,'	',1 ,0);
data_TXY = [data(:,1), data(:,3), data(:,4)];

time_since_begin = data_TXY(:,1);
for j = 1:size(data_TXY,1)
    if data_TXY(j,1) ~= -1
        time_since_begin = time_since_begin - data_TXY(j,1);
        data_TXYt = [data_TXY(j:end,:), time_since_begin(j:end,:)];
        break
    end
end    
mat_str = [folder_name, '/mat_', file_str_pre, num2str(i), '.mat'];
save(mat_str,'data_TXYt');
%save mat_str data;
end