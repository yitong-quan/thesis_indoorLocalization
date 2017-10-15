
    %% version2 ndoes oder x2, x3, x1, x5, x6
    %     locations fixed: x1(0,0)  x3(+5.12,0)
    P0 = rand(2,3)*100;
    node1and3 = [0, 5.12; 0, 0];
    options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
    [P,resnorm] = lsqnonlin(@myfun2,P0,[],[],options);
    P_all = [P(:,1), node1and3(:,2), node1and3(:,1), P(:,2), P(:,3)]; % [x2, x3, x1, x5, x6]
    %axis square;
    plot(P_all(1,:), P_all(2,:), '-*');
    daspect([10,10,10]);
    
    function F = myfun2(P3_5)
    dist2eachOther = [];
    node1and3 = [0, 5.12; 0, 0];
    P = [P3_5(:,1), node1and3(:,2), node1and3(:,1), P3_5(:,2), P3_5(:,3)];%[P1~P5: 0x2,0x3,0x1,0x5,0x6]
    for i = 1:size(P,2)
        for j = (i+1):size(P,2)
            dist2eachOther =  [dist2eachOther; norm(P(:,i) - P(:,j)) ];
        end
    end
    % [(|P1-P2|),(|P1-P3|),(|P1-P4|),(|P1-P5|),(|P2-P3|)(|P2-P4|),(|P2-P5|),(|P3-P4|),(|P3-P5|),(|P4-P5|)]
    true_dist2eachOther = [4.76, 7.45, 8.74, 7.23, 5.12, 10.0, 10.89, 6.66, 10.05, 5.40]';
    F = dist2eachOther - true_dist2eachOther;
    end