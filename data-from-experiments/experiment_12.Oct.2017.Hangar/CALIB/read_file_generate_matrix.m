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
function [trim_mean,m_a_d] = read_file_generate_matrix(experiNum)
%experimentNum = 1.7;
experimentNum = double(experiNum);

switch experimentNum
    case 1.1
        string = '1C1C_1m_t';
    case 1.2
        string = '1C1C_2m_t';
    case 1.3
        string = '1C1C_3m_t';
    case 1.4
        string = '1C1C_4m_t';
    case 1.5
        string = '1C1C_5m_t';
    case 1.6
        string = '1C1C_6m_t';
    case 1.7
        string = '1C1C_7m_t';        
    case 2.1
        string = '2020_1m_t';
    case 2.2
        string = '2020_2m_t';
    case 2.3
        string = '2020_3m_t';
    case 2.4
        string = '2020_4m_t';
    case 2.5
        string = '2020_5m_t';
    case 2.6
        string = '2020_6m_t';
    case 2.7
        string = '2020_7m_t';        
    case 3.1
        string = '3E3E_1m_t';
    case 3.2
        string = '3E3E_2m_t';
    case 3.3
        string = '3E3E_3m_t';
    case 3.4
        string = '3E3E_4m_t';
    case 3.5
        string = '3E3E_5m_t';
    case 3.6
        string = '3E3E_6m_t';
    case 3.7
        string = '3E3E_7m_t';        
    case 5.1
        string = '5A5A_1m_t';
    case 5.2
        string = '5A5A_2m_t';
    case 5.3
        string = '5A5A_3m_t';
    case 5.4
        string = '5A5A_4m_t';
    case 5.5
        string = '5A5A_5m_t';
    case 5.6
        string = '5A5A_6m_t';
    case 5.7
        string = '5A5A_7m_t';        
    case 6.1
        string = '6E6E_1m_t';
    case 6.2
        string = '6E6E_2m_t';
    case 6.3
        string = '6E6E_3m_t';
    case 6.4
        string = '6E6E_4m_t';
    case 6.5
        string = '6E6E_5m_t';
    case 6.6
        string = '6E6E_6m_t';
    case 6.7
        string = '6E6E_7m_t';        
    otherwise
        warning('please specify the experiment number #')
end
str = [string, '.log'];
fileID = fopen(str);  

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

time_diff = diff(data_t_dist(:,1));
%% below are use to remove dist with value of 0 (TODO or value smaller than 2000)
data_t_dist(data_t_dist==0) = NaN;
data_t_dist(1,1) = 0;

%{
switch experimentNum
    case {1.1 1.2 1.3 1.4 1.5 1.6 1.7}
        NodeNum = 1;
    case {2.1 2.2 2.3 2.4 2.5 2.6 2.7}
        NodeNum = 2; 
    case {3.1 3.2 3.3 3.4 3.5 3.6 3.7}
        NodeNum = 3;
    case {5.1 5.2 5.3 5.4 5.5 5.6 5.7}
        NodeNum = 5;
    case {6.1 6.2 6.3 6.4 6.5 6.6 6.7}
        NodeNum = 6;
    otherwise
        warning('please specify the experiment number #');
end
%}
[h,trim_mean,m_a_d] = my_fun(data_t_dist, experimentNum);

end
function [plot_hist,trim_mean1,m_a_d1] = my_fun(data1, experimentNumber)
    NodeNum1= floor(experimentNumber);
    columNum = NodeNum1+1;
    M = data1(:, 2);
    iii = length(M);
    while iii >0
        if M(iii,1) < 900
            M(iii,:) = [];
        end
        iii = iii-1;
    end
    figure;
    plot_hist = histogram(M, min(M):1:max(M));
    hold on; 
    trim_mean1 = trimmean(M,10);
    m_a_d1 = mad(M);
    str_title = sprintf('0x%d-%dm. trimmean:%f,  mad:%f', NodeNum1, round(10*(experimentNumber-NodeNum1)), trim_mean1, m_a_d1);
    title(str_title);
    str_fig = ['0x', num2str(floor(experimentNumber)), '_', num2str(10*(experimentNumber-floor(experimentNumber))), 'm.fig'];
    savefig(str_fig);
end
