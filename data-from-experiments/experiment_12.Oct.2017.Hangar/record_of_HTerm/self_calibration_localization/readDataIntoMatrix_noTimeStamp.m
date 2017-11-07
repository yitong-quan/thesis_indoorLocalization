%% output: data (deal with no time stamp data file)
%% 
clear;

fileID = fopen('../p6_acht_slow_t - Copy_refined.log'); 

A = textscan(fileID,'%s %c %d'); % '0x1C1C' ',' '235'
id = [A{1,1}];
dist = [A{1,3}];
%
% %time %dist1 %dist2 ... 
% %time %dist1 %dist2 ...
% ...
i1=1; i2=1; i3=1; i4=1; i5=1; i6=1;
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
    elseif strcmp(id(i),'0x5A5A')
        M(i5,5) = dist(i);
        i5 = i5 + 1;        
    elseif strcmp(id(i),'0x6E6E')
        M(i6,6) = dist(i);
        i6 = i6 + 1;
    end        
end    
fclose(fileID);

M = double(M);

temp_data = M(:,[1,3,6]);
for i = size(temp_data,1):-1:1
    for j = 1: size(temp_data,2)
        if temp_data(i,j) == 0
            temp_data(i,:) = [];
            break;
        end
    end
end


% for more than one node
data = nan(size(M,1)+3, size(M,2));
  
for jjj = 1:size(M,2)
    C = M(:,jjj);
    if any(C)
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
    %         data(end-1,jjj) = mean(C);
    %         data(end,jjj) = var(C);
            data(end-1,jjj) = trimmean(C,10);
            data(end,jjj) = mad(C);        
        end
        str_title = sprintf('0x%d. trimmean:%f,  mad:%f', jjj, data(end-1,jjj), data(end,jjj));
        subplot(2,ceil(size(M,2)/2),jjj); histogram(C, min(C):1:max(C));title(str_title);
    end
end
data(end-2:end,:)
data_T = data'; % only for making copy into Excel easier
% M = double(M);
% mean(M)
% var(M)