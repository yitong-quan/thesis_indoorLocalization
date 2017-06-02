% instrhwinfo('serial') % to find available serial ports
% instrfind
close all; clear all;
delete(instrfindall)
% s = serial('COM10'); set(s,'BaudRate',115200);
s=serial('COM24', 'BaudRate', 115200);
set(s,'Parity', 'even');
set(s,'Timeout', 1);
s.Terminator = 'CR/LF';
fopen(s);
% fprintf(s, '%s', 'setNUMBER: 1\n\r');
%out = fgetl(s);
%out = fread(s);
[out, count] = fscanf(s, '%s', 300);
[out1, count1] = fscanf(s);
[out2, count2] = fscanf(s);
fprintf(s, 'setNUMBER: 1');
[out3, count3, msg3] = fscanf(s); % out3 should be 'Write down the 16 bit IDs of 1 Nodes in HEX'
fprintf(s, 'setNODES: [0x6E6E]');
fprintf(s, 'find 0x310D, 1');
pause(1); % TODO: for the first time wake up, will need more time>> non fixed time needed

numberSend = 50;
NodeID = strings([numberSend,1]);
dist = strings([numberSend,1]);
dt = strings([numberSend,1]);
count_data = strings([2,numberSend]);
msg_data = strings([2,numberSend]);
for i = 1:numberSend
    pause(0.1); %% each loop cost time +0.6
    fprintf(s, 'find 0x310D, 1');
    [NodeID(i), count_data(1,i), msg_data(1,i)] = fscanf(s);
    [dist(i), count_data(2,i), msg_data(2,i)] = fscanf(s);
    dt(i) = datestr(now,'mmmm dd, yyyy HH:MM:SS.FFF');
end    
data = [NodeID, dist, dt];
data
fclose(s)
delete(s)
clear s
delete(instrfindall)