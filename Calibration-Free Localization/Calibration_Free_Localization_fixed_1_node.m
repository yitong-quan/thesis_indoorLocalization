clear 

%% 
% one D
%node_posi_sym = sym('n_%d', [1 2]);
node_posi_sym = sym('n_%d', [1 2]); % fix 1 out of the 2 ndoes
node_posi_sym(2) = 3;
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
u = [tag_p_sym, node_posi_sym(1:end-1)].';
Q = jacobian(b, u);
d_w_sym = Q.' * b;
% cost function
F_sym = sum(sum((f_sym).^2));

%u_0 = [-6.3 -5.9 -4 -3 -2.0 -1 0, -7 3]'; %[ tag node]
%u_0 = [-7.3 -1.9 -9.9 -2.5 -2.9 -10.9 0.9, -7.9]'; %[ tag node] fix the last node
u_0 = rand(8,1); %[ tag node] fix the last node
u_tilde = inf;
gama_init = 0.5; %<<<<<
epslon_taget = 0.00001;
iter_max = 900;
iter = 0;

u = u_0;
U = u_0;
F = inf;
cost = inf;
progress = @(iter,u,cost) fprintf('iter = %3d: u = %-32s, cost = %f\n', ...
    iter, mat2str(u,6), cost);

while iter < iter_max && cost > 0.0000001 %norm(u_tilde) >= epslon_taget
    gama = rand*gama_init;  %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< set rand factor for gamma
    d_w =  eval(subs(d_w_sym, [tag_p_sym, node_posi_sym(1:end-1)].', u));
        % todo plot the cost function : sum((f_ij)^2)
    cost = eval(subs(F_sym, [tag_p_sym, node_posi_sym(1:end-1) ].', u));
    progress(iter, u, cost);
    F = [F,  cost];
    u_tilde = gama * d_w;
    u = u - u_tilde;
    U = [U, u];
    iter = iter + 1;
    
end

plot(F, '-*r');
u'
cost
%true posi
% plot(nodes_p, zeros(size(nodes_p)), '*b');
% hold on; plot(tag_p, zeros(size(tag_p)), '*r')
