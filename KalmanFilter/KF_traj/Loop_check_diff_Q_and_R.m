

nodesNumbers = 3; % 3 or 4
i = 0.1/3; n= 0;
i_max = 10; j_max = 10;
while i <= i_max % factor_Q
    j = 0.1/3;
    while j <= j_max % factor_R
%         KF_traj_noisy_meas_3nodes(i,j);
%         KF_traj_noisy_meas_4nodes(i,j);
        KF_traj_noisy_meas_3or4nodes(i,j,nodesNumbers,true, 1); %  measurements_missing, meas_missing_rate~
        fprintf('i:%f, max:%d;      j:%f, max:%d;\n', i, i_max, j, j_max);
        j = j * 3;
        n=n+1;
    end
    i = i * 3;
end    