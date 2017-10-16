%data contain: [timeref	est_sendtimepos_x	pos_y	vel_x	vel_y sigma	valid	senderGuid]
function read_and_generate_TXY_matrix_from_ultra_sound_system(i)
%M = dlmread(filename,delimiter,R1,C1) starts reading at row offset R1 and column offset C1. For example, the offsets R1=0, C1=0 specify the first value in the file.
file_str_pre = 'positions';
file_name = [file_str_pre, num2str(i), '.log'];

data  = dlmread(file_name,'	',1 ,0);
data_TXY = [data(:,1), data(:,3), data(:,4)];

mat_str = ['mat_', file_str_pre, num2str(i), '.mat'];
save(mat_str,'data_TXY');
%save mat_str data;
end