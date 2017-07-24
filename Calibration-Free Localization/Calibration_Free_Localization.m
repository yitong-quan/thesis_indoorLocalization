clear 
%% 
% one D
node_posi_sym = sym('n_%d', [1 2]);
tag_p_sym = sym('tag_p_%d', [1 7]);
f_sym = sym('f_%d', [length(node_posi_sym), length(tag_p_sym)]);
nodes_p = [-5, 3];
tag_p = linspace(-6,0,7);
true_dist = [abs(tag_p - nodes_p(1)) ; abs(tag_p - nodes_p(2))];
for i = 1:length(node_posi_sym)
    for j = 1:length(tag_p_sym)
        f_sym(i,j) = sqrt((node_posi_sym(i) - tag_p_sym(j))^2) - true_dist(i,j);
    end
end    

b = [];
for l = 1:size(f_sym)
    b = [b f_sym(l,:)];
end    
b = b';

u = [tag_p_sym node_posi_sym]';
Q = jacobian(b, u);
d_w = Q' * b;