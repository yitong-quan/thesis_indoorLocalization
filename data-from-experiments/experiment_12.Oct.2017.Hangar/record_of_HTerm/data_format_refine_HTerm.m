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
experimentNum = 01.1;
last_time = '00:00:00.000:';

switch experimentNum
    case 01
        fileID_r = fopen('1-CENTER_t - Copy.log', 'r');
        fileID_w =fopen('1-CENTER_t - Copy_refined.log', 'w');   
    case 01.1
        fileID_r = fopen('1.1-CENTER_t - Copy.log', 'r');
        fileID_w =fopen('1.1-CENTER_t - Copy_refined.log', 'w'); 
    case 1
        fileID_r = fopen('p1_circle_t - Copy.log', 'r');
        fileID_w =fopen('p1_circle_t - Copy_refined.log', 'w');
    case 3
        fileID_r = fopen('p3_circle_t - Copy.log', 'r');
        fileID_w =fopen('p3_circle_t - Copy_refined.log', 'w');
    case 4
        fileID_r = fopen('p4_circle_t - Copy.log', 'r');
        fileID_w =fopen('p4_circle_t - Copy_refined.log', 'w');
    case 5
        fileID_r = fopen('p5_acht_t - Copy.log', 'r');
        fileID_w =fopen('p5_acht_t - Copy_refined.log', 'w');
    case 6
        fileID_r = fopen('p6_acht_slow_t - Copy.log', 'r');
        fileID_w =fopen('p6_acht_slow_t - Copy_refined.log', 'w');
    case 7
        fileID_r = fopen('p7_random_mSpeed_t - Copy.log', 'r');
        fileID_w =fopen('p7_random_mSpeed_t - Copy_refined.log', 'w');
    case 8
        fileID_r = fopen('p8_random_sSpeed_t - Copy.log', 'r');
        fileID_w =fopen('p8_random_sSpeed_t - Copy_refined.log', 'w');
    case 9
        fileID_r = fopen('p9_random_fSpeed_t - Copy.log', 'r');
        fileID_w =fopen('p9_random_fSpeed_t - Copy_refined.log', 'w');
    case 10
        fileID_r = fopen('p10_flow_sSpeed_t - Copy.log', 'r');
        fileID_w =fopen('p10_flow_sSpeed_t - Copy_refined.log', 'w');
    case 11
        fileID_r = fopen('p11_3d_random_t - Copy.log', 'r');
        fileID_w =fopen('p11_3d_random_t - Copy_refined.log', 'w');
    case 14
        fileID_r = fopen('p14_random_sSpeed_t - Copy.log', 'r');
        fileID_w =fopen('p14_random_sSpeed_t - Copy_refined.log', 'w');
    otherwise
        warning('please specify the experiment number #')
end

tline = fgetl(fileID_r); % format char
time_format = '^[0-9]+:[0-9]+:[0-9]+\.[0-9]+';
flag_combination = 0;
while ischar(tline)
    if regexp(tline, time_format) == 1 % begin with 'time_format'
        %disp(tline);
        this_time = tline;
        %time_interval = this_time - last_time;
        time_interval = etime(datevec(this_time),datevec(last_time));
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


