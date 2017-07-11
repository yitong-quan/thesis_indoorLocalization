%% output: data
%%
clear;
%fileID = fopen('0x1C1C-10m.log'); %'0x2020, 0x3E3E, 0x4D4D, 0x6E6E-2m.log'
fileID = fopen('0x4D4D-3m.log'); 
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