% instrhwinfo('serial') % to find available serial ports
% instrfind
close all; clear all;
delete(instrfindall)

% set serial port properties
%s = serial('COM10'); set(s,'BaudRate',115200);
%s=serial('COM24', 'BaudRate', 115200);
s=serial('COM4', 'BaudRate', 115200); % small PC
set(s,'Parity', 'even');
set(s,'Timeout', 0.7); % TODO: adjust. sampling rate is 1.5, 0.66s <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
s.Terminator = 'CR/LF';
fopen(s);
%[out, count] = fscanf(s, '%s', 300); [out1, count1] = fscanf(s); [out2, count2] = fscanf(s);

% set nodeNumber & nodeID
nodeNumber = 1; %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
fprintf(s, 'setNUMBER: %d\n', nodeNumber); % '\n' is important here, since... 
% ...'%s'Specify a format for the data that does not include the terminator, or configure the terminator to empty.  
[out3, count3, msg3] = fscanf(s); % out3 should be 'Write down the 16 bit IDs of 1 Nodes in HEX'
%fprintf(s, 'setNODES: [0x1C1C, 0x2020, 0x3E3E, 0x4D4D, 0x6E6E]'); %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
fprintf(s, 'setNODES: [0x1C1C]');
%fprintf(s, 'setNODES: [0x1C1C, 0x2020, 0x3E3E]');
FindCommand = 'find 0x310D, 1'; %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
fprintf(s, FindCommand);
[out0, count0, msg0] = fscanf(s);
pause(2); % TODO: for the first time wake up, will need more time>> non fixed time needed

numberSend = 100;

% construct the data
id_dist = strings([numberSend,nodeNumber]); % returns an m-by-n array of strings with no characters
%dist = strings([numberSend,nodeNumber]);
timeStamp = strings([numberSend,1]);
count_data = strings([numberSend,nodeNumber]); % count_data = strings([2,numberSend,nodeNumber]);
msg_data = strings([numberSend,nodeNumber]); % msg_data = strings([2,numberSend,nodeNumber]);

% send out 'find' command, record and process data from the basis station
for i = 1:numberSend
    fprintf('i=%d ', i); %disp(i);%i %display(i);
    %pause(0.01); %% each loop cost time +0.6
    fprintf(s, FindCommand);
    timeStamp(i) = datestr(now,'SS.FFF');
    %timeStamp(i) = datestr(now,'HH:MM:SS.FFF mmm dd yyyy');
    for j = 1:1
    % for j = 1:nodeNumber
        %pause(0.02);
        set(s,'Timeout', 0.7); 
        [id_dist(i,j), count_data(i,j), msg_data(i,j)] = fscanf(s);
        % add this if() to keep the data structure in right order when the
        % communication with nodes are not achieved
        if (msg_data(i,j) == 'A timeout occurred before the Terminator was reached.')
            pause(0.05);
            set(s,'Timeout', 0.05); 
            [id_dist(i,j), count_data(i,j), msg_data(i,j)] = fscanf(s);
            if (msg_data(i,j) == 'A timeout occurred before the Terminator was reached.')
                pause(0.05);
                [id_dist(i,j), count_data(i,j), msg_data(i,j)] = fscanf(s);
                if (msg_data(i,j) == 'A timeout occurred before the Terminator was reached.')
                    pause(0.05);
                    id_dist(i,j) = fscanf(s);
                end
            end
        end
        %[dist(i,j), count_data(2,i,j), msg_data(2,i,j)] = fscanf(s);
    end
end    
data = [id_dist, timeStamp];
data = regexprep(data,'\r\n|\n|\r',''); % remove '\r\n' from all the strings
% distance with double format
%dist = str2double(data(:,3:4)); 
fclose(s)
delete(s)
clear s
delete(instrfindall)

%% for testing
time = str2double(timeStamp);
timeInterval = diff(time);
timeInterval_mean = (sum(timeInterval)- min(timeInterval)-max(timeInterval))/(length(timeInterval)-2);