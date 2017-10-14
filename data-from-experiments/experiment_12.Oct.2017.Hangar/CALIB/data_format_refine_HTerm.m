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
function data_format_refine_HTerm(experiNumber)
experimentNum = double(experiNumber);
last_time = '00:00:00.000:';

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
        warning('please specify the experiment number #');
        return;
end
str = [string, ' - Copy.log'];
fileID_r = fopen(str, 'r');
str = [string, ' - Copy_refined.log'];
fileID_w =fopen(str, 'w');   


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
end

