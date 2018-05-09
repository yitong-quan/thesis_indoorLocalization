%% nodes positions are recoverd by using optimization method,
%  since the positions of some nodes are outside the rang of the
%  MotionCapture system .
    %% version2 nodes oder 0x2, 0x3, 0x1, 0x5, 0x6
    %     locations fixed: x1(0,0)  x3(+5.22,0)
    P0 = rand(2,3)*1;
    node1and3 = [0, 5.22; 0, 0];
    options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
    [P,resnorm] = lsqnonlin(@myfun2,P0,[],[],options);
    P_all = [P(:,1), node1and3(:,2), node1and3(:,1), P(:,2), P(:,3)]; % [x2, x3, x1, x5, x6]
    %axis square;
    figure; 
    plot(P_all(1,:), P_all(2,:), '-*');
    str_title = ['resnorm: ', num2str(resnorm)];
    title(str_title);
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
    %%%[(|P1-P2|),(|P1-P3|),(|P1-P4|),(|P1-P5|),(|P2-P3|)(|P2-P4|),(|P2-P5|),(|P3-P4|),(|P3-P5|),(|P4-P5|)]
    %%without the radium of the pillar: 6cm, leads to Residual of 0.000843665
    true_dist2eachOther = [489, 742, 889, 694, 522, 1071, 1104, 731, 1018, 542]' /100; % these distances are determined by hand using laser-meter 
    F = dist2eachOther - true_dist2eachOther;
    end
    