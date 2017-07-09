%% pseudocode
last_time = 00:00:00.000;
read line_by_line
    if (time_format)
        this_time = read_time_of_this_line;
        time_interval = this_time - last_time;
        if time_interval < 0.3
            flag_combination = 1; <<<<<<<<<<<<<<<<<<<<<<<<
            do_nothing;
        else
            flag_combination = 0;
            write_this_time_to_file(begin with '\n');
        end
        last_time = this_time;
    elseif (begine_with '0x')
        write_this_time_to_file(begin with '\n');
    elseif (empty line)
        write_empty_line_to_file;
    else % begin with something else than 'time_format' or '0x'
        if flag_combination == 1
            write_this_time_to_file(begin without '\n');
        end
    end
    