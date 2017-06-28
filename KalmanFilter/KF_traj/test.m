%% 

% % test for plot color shade
% A = 1:0.1:20;
% B = sin (A);
% plot(A,B);

% % test for randomly generate NaN in a matrix
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

% % test for randperm(Random permutation)
% clear all; close all;
% index1_NaN = randperm(6,3); % in this case, number from 1 to 6 will be all there, which is not what I want
% figure;histogram(index1_NaN,6);

% test for H matrix with missing data
nodes_Nums = 3;
    syms N_xy x_m Z_e
    numNodes = nodes_Nums; % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    N_x_n = sym('N_x_n%d%d', [2 numNodes]); % posNodes %[n_x1 n_x2 n_x3; n_y1 n_y2 n_y3]. i.e. 'N_x_n21' means the y_posi of the ist node 
    x_m = sym('x_m%d', [4 1]); % time_updated state vector
    if  nodes_Nums == 4
            Z_e = [     % Z_e: expected measurements [numNodes 1]<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
           sqrt( (N_x_n(1,1) - x_m(1))^2 + (N_x_n(2,1) - x_m(2))^2 );
           sqrt( (N_x_n(1,2) - x_m(1))^2 + (N_x_n(2,2) - x_m(2))^2 );
           sqrt( (N_x_n(1,3) - x_m(1))^2 + (N_x_n(2,3) - x_m(2))^2 );
           sqrt( (N_x_n(1,4) - x_m(1))^2 + (N_x_n(2,4) - x_m(2))^2 )
           ];
    end
    if  nodes_Nums == 3
            Z_e = [     % Z_e: expected measurements [numNodes 1]<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            sqrt( (N_x_n(1,1) - x_m(1))^2 + (N_x_n(2,1) - x_m(2))^2 );
            sqrt( (N_x_n(1,2) - x_m(1))^2 + (N_x_n(2,2) - x_m(2))^2 );
            sqrt( (N_x_n(1,3) - x_m(1))^2 + (N_x_n(2,3) - x_m(2))^2 )
            ];
    end
    Z_e = simplify(Z_e);
    H_symbolic = jacobian(Z_e, x_m)
    size(H_symbolic)