%% output: data_t_dist : (1.time, 2.3.4.5...dist of nodes used)
%% Pseudocode 
%{ 
read i_th line >> content
	if time_format
        package(i).time_stamp = content
    end
    while !time_format
        [ID,delimiter,dist] = textread(content,'%s %c %d')
        package(i).ID = [package(i).ID ID]
        package(i).dist = [package(i).dist dist]
    end
%}
%%
clear;
%fileID = fopen('6th_3D_output_2017-07-07_15-06-08_copy_refined.log'); 
fileID = fopen('3m_all_t.log'); 


%{
% tline = fgetl(fid);
% while ischar(tline)
%     disp(tline);
%     tline = fgetl(fid);
% end

% formatSpec = '%s, %d';
% sizeA = [2 Inf];
% A1 = fscanf(fileID,formatSpec,sizeA)
%}
time_format = '^[0-9]+:[0-9]+:[0-9]+\.[0-9]+';
i = 0; % package index
tline = fgetl(fileID); % format char
while ischar(tline)
    % disp('!');disp(tline);disp('!');
    if ~isempty(tline)
        if regexp(tline, time_format) == 1 % begin with 'time_format'
            i = i+1;
            package(i).time_stamp = datevec(tline);
            package(i).ID = [];
            package(i).dist = [];
        else
            A = textscan(tline,'%s %c %d'); % '0x1C1C' ',' '235'
            ID = [A{1,1}];
            dist = [A{1,3}];
            package(i).ID = [package(i).ID ID];
            package(i).dist = [package(i).dist dist];
        end
    end
    tline = fgetl(fileID);
end
time_starting_ponit = package(1).time_stamp;
for j = 1:length(package)
    package(j).timeFromStartingPointInSec = etime(package(j).time_stamp, time_starting_ponit);
end
data_t_dist = zeros(length(package), 1+length(package(1).dist));
for ii = 1:length(package)
    data_t_dist(ii,:) = [package(ii).timeFromStartingPointInSec, double(package(ii).dist)];
end
disp('ID used are followed: ');
disp(package(1).ID);
if length(package(1).ID) < 5
    warning('Warning. package#<5, not all the nodes are used, please change code and add in a column to the coresponding ndoes')
    %base on the nodes used, add up columns for data 'from' nonused nodes
	data_t_dist = [data_t_dist(:,1:2)  nan(length(data_t_dist),1) data_t_dist(:,3:end)];
end    

data_t_dist_full_raw = data_t_dist;
x1C1C_raw = (data_t_dist(:,2)  + 5)/ 100 / 0.9925;
data_t_dist_full_raw(:,2) = x1C1C_raw;
x2020_raw = (data_t_dist(:,3)  + 0) / 100 / 0.9993;
data_t_dist_full_raw(:,3) = x2020_raw;
x3E3E_raw = (data_t_dist_full_raw(:,4)  + 14) / 100 / 1.00547;
data_t_dist_full_raw(:,4) = x3E3E_raw;
x4D4D_raw = (data_t_dist_full_raw(:,5)   + 8) / 100 / 1.01452;
data_t_dist_full_raw(:,5) = x4D4D_raw;
x6E6E_raw = (data_t_dist_full_raw(:,6)   + 0) / 100 / 0.9993;
data_t_dist_full_raw(:,6) = x6E6E_raw;
data_t_dist_full_raw(data_t_dist_full_raw <= 0) = NaN;
data_t_dist_full_raw(1,1) = 0;

data_t_dist_calied = data_t_dist_full_raw;
data_t_dist_calied(:,2) = data_t_dist_full_raw(:,2)*89.73+25.4;
data_t_dist_calied(:,3) = data_t_dist_full_raw(:,3)*90.42+5.531;
data_t_dist_calied(:,4) = data_t_dist_full_raw(:,4)*91.91+23.44;
data_t_dist_calied(:,5) = data_t_dist_full_raw(:,5)*89.84+21.37;
data_t_dist_calied(:,6) = data_t_dist_full_raw(:,6)*91.74+15.89;
data = nan(size(data_t_dist_calied,1)+3, size(data_t_dist_calied,2));
  
for jjj = 1:size(data_t_dist_calied,2)
    C = data_t_dist_calied(:,jjj);
    iii = length(C);
    while iii >0
        if C(iii) < 200 || isnan(C(iii))
            C(iii) = [];
        end
        iii = iii -1;
    end
    if length(C) > 0  
        C = double(C);
        data(1:length(C),jjj) = C;
        data(end-1,jjj) = mean(C);
        data(end,jjj) = var(C);
%         data(end-1,jjj) = trimmean(C,10);
%         data(end,jjj) = mad(C);        
    end
    str_title = sprintf('0x%d. trimmean:%f,  mad:%f', jjj, data(end-1,jjj), data(end,jjj));
    figure; hist(C);title(str_title);
end
data(end-2:end,:)
% save('data_t_dist_HTerm_temp.mat','data_t_dist_calied');
% disp('data_t_dist_calied are saved as ,data_t_dist_HTerm_temp.mat, ad the same folder');

