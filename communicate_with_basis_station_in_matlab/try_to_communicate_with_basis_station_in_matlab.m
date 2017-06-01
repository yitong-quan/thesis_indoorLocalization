% instrhwinfo('serial') % to find available serial ports
% instrfind
close all; clear all;
s = serial('COM10');
set(s,'BaudRate',115200);
fopen(s);
fprintf(s,'setNUMBER: 1')
out = fread(s);
%out = fscanf(s);
fclose(s)
delete(s)
clear s