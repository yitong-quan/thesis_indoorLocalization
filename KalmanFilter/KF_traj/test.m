%% 

%% test for plot color shade
% A = 1:0.1:20;
% B = sin (A);
% plot(A,B);

%% test for randomly generate NaN in a matrix
% clear all; close all;
% numberOfMissing = 3;
% A = ones(4,15);
% ind = randi([1 size(A,1)] ,numberOfMissing,size(A,2));
% figure;histogram(ind,size(ind,2));
% for i = 1:size(A,2)
%     for j = 1: numberOfMissing
%         A(ind(j,i),i) = NaN;
%     end
% end    

%% test for randperm(Random permutation)
% clear all; close all;
% index1_NaN = randperm(6,3); % in this case, number from 1 to 6 will be all there, which is not what I want
% figure;histogram(index1_NaN,6);

%% test for H matrix with missing data
% nodes_Nums = 3;
%     syms N_xy x_m Z_e
%     numNodes = nodes_Nums; % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%     N_x_n = sym('N_x_n%d%d', [2 numNodes]); % posNodes %[n_x1 n_x2 n_x3; n_y1 n_y2 n_y3]. i.e. 'N_x_n21' means the y_posi of the ist node 
%     x_m = sym('x_m%d', [4 1]); % time_updated state vector
%     if  nodes_Nums == 4
%             Z_e = [     % Z_e: expected measurements [numNodes 1]<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%            sqrt( (N_x_n(1,1) - x_m(1))^2 + (N_x_n(2,1) - x_m(2))^2 );
%            sqrt( (N_x_n(1,2) - x_m(1))^2 + (N_x_n(2,2) - x_m(2))^2 );
%            sqrt( (N_x_n(1,3) - x_m(1))^2 + (N_x_n(2,3) - x_m(2))^2 );
%            sqrt( (N_x_n(1,4) - x_m(1))^2 + (N_x_n(2,4) - x_m(2))^2 )
%            ];
%     end
%     if  nodes_Nums == 3
%             Z_e = [     % Z_e: expected measurements [numNodes 1]<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%             sqrt( (N_x_n(1,1) - x_m(1))^2 + (N_x_n(2,1) - x_m(2))^2 );
%             sqrt( (N_x_n(1,2) - x_m(1))^2 + (N_x_n(2,2) - x_m(2))^2 );
%             sqrt( (N_x_n(1,3) - x_m(1))^2 + (N_x_n(2,3) - x_m(2))^2 )
%             ];
%     end
%     Z_e = simplify(Z_e);
%     H_symbolic = jacobian(Z_e, x_m)
%     size(H_symbolic)
    
    
%     
% %% test for  error_ellipse
% clear all;
% close all;
% 
% % Mean and Covariance 
% avg = [ 0 0 ]
% covariance =  [ 0.02   0
%                 0   0.02 ]
% 
% [eigenvec, eigenval] = eig(covariance)
% 
% % Get the index of the largest eigenvector
% [max_evc_ind_c, r] = find(eigenval == max(max(eigenval)));
% max_evc = eigenvec(:, max_evc_ind_c);
% 
% % Get the largest eigenvalue
% max_evl = max(max(eigenval));
% 
% % Get the smallest eigenvector and eigenvalue
% if(max_evc_ind_c == 1)
%     min_evl = max(eigenval(:,2))
%     min_evc = eigenvec(:,2);
% else
%     min_evl = max(eigenval(:,1))
%     min_evc = eigenvec(1,:);
% end
% 
% % Calculate the angle between the x-axis and the largest eigenvector
% angle = atan2(max_evc(2), max_evc(1));
% 
% % This angle is between -pi and pi.
% % Let's shift it such that the angle is between 0 and 2pi
% if(angle < 0)
%     angle = angle + 2*pi;
% end
% 
% 
% % Get the 95% confidence interval error ellipse
% chisquare_val = 2.4477;
% theta_grid = linspace(0,2*pi);
% phi = angle;
% X0  = mean(1);
% Y0  = mean(2);
% a   = chisquare_val*sqrt(max_evl);
% b   = chisquare_val*sqrt(min_evl);
% 
% % the ellipse in x and y coordinates 
% ellipse_x_r  = a*cos( theta_grid );
% ellipse_y_r  = b*sin( theta_grid );
% 
% %Define a rotation matrix
% R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
% 
% %let's rotate the ellipse to some angle phi
% r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;
% 
% % Draw the error ellipse
% figure;
% plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,'-')
% hold on;
% 
% % Plot the original data
% %figure;
% %plot(data(:,1), data(:,2), '.');
% %mindata = min(min(data));
% %maxdata = max(max(data));
% %xlim([mindata-3, maxdata+3]);
% %ylim([mindata-3, maxdata+3]);
% %hold on;
% 
% % Plot the eigenvectors
% figure;
% quiver(X0, Y0, max_evc(1)*sqrt(max_evl), max_evc(2)*sqrt(max_evl), '-m', 'LineWidth',2);
% hold on;
% quiver(X0, Y0, min_evc(1)*sqrt(min_evl), min_evc(2)*sqrt(min_evl), '-g', 'LineWidth',2);
% 
% % Set the axis labels
% hXLabel = xlabel('x');
% hYLabel = ylabel('y');
% 
% 
% pause()

%% test for delete random numbers of elements in each set
z_all = randn(4,1000);
            for kk = 1:size(z_all,2)  %<<<<<<<<<<<<<<<<<<<<NOT GOOD TODO: IMPROVE
                num_NaN_in_each_column = randi([0 size(z_all,1)], 1,1);
                index_NaN_in_each_column = randi([1 size(z_all,1)] ,1, num_NaN_in_each_column);
                %figure;histogram(index_NaN_in_each_column, 4);
                for ll = 1: size(index_NaN_in_each_column,2)
                    z_all(index_NaN_in_each_column(ll),kk) = NaN;
                end
            end