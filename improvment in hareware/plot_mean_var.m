x_idx = [0,1,2,3,4,5];
e=errorbar(x_idx,mean_before,var_before);
xlim([-0.5 5.5]);
e.Marker = '*';
e.CapSize = 20;
 hold on;
 
 e1=errorbar(x_idx,mean_after,var_after);
 e1.Marker = '*';
 e1.CapSize = 20;
 
 legend('mean var, before', 'mean var, after')
 
 xlabel('num of usefule measurments each set');
 ylabel('ratio to total');