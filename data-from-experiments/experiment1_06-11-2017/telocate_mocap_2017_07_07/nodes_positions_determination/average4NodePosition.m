function node_position = average4NodePosition(startTime, EndTime, whole_trajectory)
    ind_strart = find(whole_trajectory(:,1) == startTime);
    ind_end = find(whole_trajectory(:,1) == EndTime);
    traj_segment = whole_trajectory(ind_strart:ind_end, :);
    % remove zeros elements
    for i = length(traj_segment) : -1 :1
        if (traj_segment(i,2) == 0 && traj_segment(i,3) == 0 && traj_segment(i,4) == 0)
            traj_segment(i,:) = [];
        end
    end
    if ~all(all(traj_segment)) % if content zero
        warning('content zero elements, please check the traj')
    end
    node_position = mean(traj_segment(:, 2:4));
end