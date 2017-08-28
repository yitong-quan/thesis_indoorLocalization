

nodesNumbers = 4; % 3 or 4
n= 0;
% i_max = 100; 
j_max = 1;
%while i <= i_max % factor_Q
% while i <=0.1/3 % factor_Q    
    i = 1;  % i should be 1
    j = 0.1/3;
    while j <= j_max % factor_R
    %while j <= 0.1/3 % factor_R
%         KF_traj_noisy_meas_3nodes(i,j);
%         KF_traj_noisy_meas_4nodes(i,j);
        KF_traj_noisy_meas_3or4nodes(i,j,nodesNumbers,false, 0); %  measurements_missing, meas_missing_rate~
        fprintf('i:%f;      j:%f, j_max:%d;\n', i, j, j_max);
        j = j * 3;
        n=n+1;
    end
%     i = i * 3;
% end    