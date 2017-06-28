fileID = fopen('0x1C1C-2m.log');

% tline = fgetl(fid);
% while ischar(tline)
%     disp(tline);
%     tline = fgetl(fid);
% end

% formatSpec = '%s, %d';
% sizeA = [2 Inf];
% A1 = fscanf(fileID,formatSpec,sizeA)

A = textscan(fileID,'%s %c %d'); % '0x1C1C' ',' '235'
%
% %dist1 %dist2 ... %time 
% %dist1 %dist2 ... %time
% ...
for i = 1:size(A)
    if A(i,1) == '0x1C1C'
        
    end        
end    
fclose(fileID);