for i = 1:10:21
    for j = 2:10:12
        figure();
        KF_traj_noisy_meas_3nodes(i,j);
        %printf('i:%f; j:%f;\n',i,j)
    end
end    