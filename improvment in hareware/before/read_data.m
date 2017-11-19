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
function read_data(experiNumber)
experimentNum = experiNumber;

switch experimentNum
    case 3.1
        string = '3_1.txt';  
    case 3
        string = '3.txt';  
    case 2
        string = '2.txt';  
    case 5
        string = '5.txt';  
    case 6
        string = 'imudata2 (1) - Copy.txt';  
    case 7
        string = 'p7_random_mSpeed_t';  
    case 8
        string = 'p8_random_sSpeed_t';  
    case 9
        string = 'p9_random_fSpeed_t';  
    case 10
        string = 'p10_flow_sSpeed_t';  
    case 11
        string = 'p11_3d_random_t';   
    case 14
        string = 'p14_random_sSpeed_t';  
    otherwise
        warning('please specify the experiment number #')
        return
end
str = [string];
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
i = 1; % package index
            package(i).ID = [];
            package(i).dist = [];
tline = fgetl(fileID); % format char
while ischar(tline)
    % disp('!');disp(tline);disp('!');
    if ~isempty(tline)
        if regexp(tline, time_format) == 1 % begin with 'time_format'
            package(i).time_stamp = datevec(tline);
            i = i+1;
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
for j = 1:length(package)-1
    package(j).timeFromStartingPointInSec = etime(package(j).time_stamp, time_starting_ponit);
end
data_t_dist = zeros(length(package), 1+length(package(1).dist));
for ii = 1:length(package)-1
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
d_dist = data_t_dist(:,2:end);
d_dist(d_dist < 500) = NaN;
data_t_dist = [data_t_dist(:,1), d_dist];
data_t_dist =data_t_dist(1:end-1,:);
str_data = ['data_t_dist_', string, '.mat'];
save(str_data,'data_t_dist');
end
%{
for jjj = 1:size(data_t_dist_calied,2)
    C = data_t_dist_calied(:,jjj);
    iii = length(C);
    while iii >0
        if C(iii) < 2000 || isnan(C(iii))
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
%}

%{
A = textscan(fileID,'%s %c %d'); % '0x1C1C' ',' '235'
id = [A{1,1}];
dist = [A{1,3}];
%
% %time %dist1 %dist2 ... 
% %time %dist1 %dist2 ...
% ...
i1=1; i2=1; i3=1; i4=1; i6=1;
for i = 1:size(id)
    if strcmp(id(i),'0x1C1C') 
        M(i1,1) = dist(i);
        i1 = i1 + 1;
    elseif strcmp(id(i),'0x2020') 
        M(i2,2) = dist(i);
        i2 = i2 + 1;
    elseif strcmp(id(i),'0x3E3E') 
        M(i3,3) = dist(i);
        i3 = i3 + 1;
    elseif strcmp(id(i),'0x4D4D') 
        M(i4,4) = dist(i);
        i4 = i4 + 1;
    elseif strcmp(id(i),'0x6E6E')
        M(i6,5) = dist(i);
        i6 = i6 + 1;
    end        
end    
fclose(fileID);

nn=0; 
for ii = 1:size(M,1)
    for jj = 1:size(M,2)
        if M(ii,jj) < 200
            sprintf('suspicious index [%d %d]', ii, jj)
            nn = nn+1;
        end
    end
end

% for one node
%{
iii = size(M,1); jjj=0;
while iii >0 
        if M(iii,1) < 200
            M(iii,:) = [];
            jjj = jjj+1;
        end
        iii = iii-1;
end
%}

% for more than one node
data = nan(size(M,1)+3, size(M,2));
  
for jjj = 1:size(M,2)
    C = M(:,jjj);
    iii = length(C);
    while iii >0
        if C(iii) < 200
            C(iii) = [];
        end
        iii = iii -1;
    end
    if length(C) > 0  
        C = double(C);
        data(1:length(C),jjj) = C;
        data(end-1,jjj) = mean(C);
        data(end,jjj) = var(C);
    end
end
% M = double(M);
% mean(M)
% var(M)
%}