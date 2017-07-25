clear 
%% 
% one D
node_posi_sym = sym('n_%d', [1 2]);
tag_p_sym = sym('tag_p_%d', [1 7]);
f_sym = sym('f_%d', [length(node_posi_sym), length(tag_p_sym)]);
nodes_p = [-7, 3];
tag_p = linspace(-6,0,7);
true_dist = [abs(tag_p - nodes_p(1)) ; abs(tag_p - nodes_p(2))];
for i = 1:length(node_posi_sym)
    for j = 1:length(tag_p_sym)
        f_sym(i,j) = abs(node_posi_sym(i) - tag_p_sym(j)) - true_dist(i,j);
    end
end    
b = [];
for l = 1:size(f_sym)
    b = [b f_sym(l,:)];
end    
b = b.';
u = [tag_p_sym node_posi_sym].';
Q = jacobian(b, u);
d_w_sym = Q.' * b;

u_0 = [-6.5 -5.5 -4.3 -3.1 -2.0 -0.9 0 -7 3]';
u_tilde = inf;
gama = 0.005; %<<<<<
epslon_taget = 0.01;
iter_max = 1000;
iter = 1;

u = u_0;
U = u_0;
while iter < iter_max && norm(u_tilde) >= epslon_taget
    d_w =  eval(subs(d_w_sym, [node_posi_sym tag_p_sym].', u));
    u_tilde = gama * d_w;
    u = u - u_tilde;
    U = [U u];
    iter = iter + 1;
end
%true posi
plot(nodes_p, zeros(size(nodes_p)), '*b');
hold on; plot(tag_p, zeros(size(tag_p)), '*r')
