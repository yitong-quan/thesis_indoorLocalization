version = 2; % 1 or 2
if version == 2
    %% version2 ndoes oder x2, x3, x1, x5, x6
    %     locations fixed: x1(0,0)  x3(+6,0)
    P0 = rand(2,3)*100;
    node1and3 = [0, 5.12; 0, 0];
    options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
    [P,resnorm] = lsqnonlin(@myfun2,P0,[],[],options);
    P_all = [P(:,1), node1and3(:,2), node1and3(:,1), P(:,2), P(:,3)];
    %axis square;
    plot(P_all(1,:), P_all(2,:), '-*');
    daspect([10,10,10]);
    
else
    %% version1 ndoes oder x2, x3, x1, x5, x6
    %     locations fixed: x2(0,0)  x3(-6,0)
    P0 = rand(2,3);
    node1and2 = [0, -6; 0, 0];
    
    options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
    [P,resnorm] = lsqnonlin(@myfun,P0,[],[],options);
    P_all = [node1and2 , P];
    %axis square;
    plot(P_all(1,:), P_all(2,:), '-or');
    daspect([10,10,10]);
end
    
    function F = myfun2(P3_5)
    dist2eachOther = [];
    node1and3 = [0, 4.7; 0, 0];
    P = [P3_5(:,1), node1and3(:,2), node1and3(:,1), P3_5(:,2), P3_5(:,3)];
    for i = 1:size(P,2)
        for j = (i+1):size(P,2)
            dist2eachOther =  [dist2eachOther; norm(P(:,i) - P(:,j)) ];
        end
    end
    true_dist2eachOther = [6, 7.85, 8.77, 6.47, 4.7, 10, 11.14, 6.6, 10.2, 6.04]';
    F = dist2eachOther - true_dist2eachOther;
    end
    
    function F = myfun(P3_5)
    dist2eachOther = [];
    node1and2 = [0, -6; 0, 0];
    P = [node1and2 , P3_5];
    for i = 1:size(P,2)
        for j = (i+1):size(P,2)
            dist2eachOther =  [dist2eachOther; norm(P(:,i) - P(:,j)) ];
        end
    end
    true_dist2eachOther = [6, 7.85, 8.77, 6.47, 4.7, 10, 11.14, 6.6, 10.2, 6.04]';
    F = dist2eachOther - true_dist2eachOther;
    end