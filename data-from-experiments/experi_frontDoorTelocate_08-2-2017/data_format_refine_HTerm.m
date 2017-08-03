%%refinde the file when Time stamp is present
%% pseudocode
%{
% import_file;
% last_time = 00:00:00.000;
% read line_by_line
%     if (time_format)
%         this_time = read_time_of_this_line;
%         time_interval = this_time - last_time;
%         if time_interval < 0.3
%             flag_combination = 1; <<<<<<<<<<<<<<<<<<<<<<<<
%             do_nothing;
%         else
%             flag_combination = 0;
%             write_this_time_to_file(begin with '\n');
%         end
%         last_time = this_time;
%     elseif (begine_with '0x')
%         write_this_time_to_file(begin with '\n');
%     elseif (empty line)
%         write_empty_line_to_file;
%     else % begin with something else than 'time_format' or '0x'
%         if flag_combination == 1
%             write_this_time_to_file(begin without '\n');
%         end
%     end
%}
%%
clear;
last_time = '00:00:00.000:';
fileID_r = fopen('fixed_13456_t_copy.log', 'r');
fileID_w = fopen('fixed_13456_t_copy_refined.log', 'w');
tline = fgetl(fileID_r); % format char
time_format = '^[0-9]+:[0-9]+:[0-9]+\.[0-9]+';
flag_combination = 0;
while ischar(tline)
    if regexp(tline, time_format) == 1 % begin with 'time_format'
        %disp(tline);
        this_time = tline;
        %time_interval = this_time - last_time;
        time_interval = etime(datevec(this_time),datevec(last_time))
        if time_interval < 0.3
            flag_combination = 1; % <<<<<<<<<<<<<<<<<<<<<<<<
        else
            flag_combination = 0;
           % write_this_time_to_file(begin with '\n');
            fprintf(fileID_w, '\r\n\r\n%s', tline);
        end
        last_time = this_time;
    elseif  strncmp(tline, '0x', 2) % (begine_with '0x')
        fprintf(fileID_w, '\r\n%s', tline);
    elseif  strncmp(tline, '0', 1) && flag_combination == 0% (begine_with '0x')
        fprintf(fileID_w, '\r\n%s', tline);
    else % begin with something else than 'time_format' or '0x'
        if flag_combination == 1
            fprintf(fileID_w, '%s', tline);
        end
    end
    % date3 = '12:16:42.396'; date2 = '12:16:41.396'; etime(datevec(date2),datevec(date1))
    % disp(tline)
    tline = fgetl(fileID_r);
end
fclose(fileID_w);
fclose(fileID_r);


