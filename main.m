%% MDH Table

syms q1 q2 q3 q4

syms a2 a3
a2 = 0.46;      % m
a3 = 0.44;      % m

MDH_Table = [0,     0,     0,     q1;...
            pi/2,   0,     0,     q2;...
            0,      a2,    0,     q3;...
            0,      a3,    0,     q4];


indices = (1:4)';
alphas = MDH_Table(:, 1);
as = MDH_Table(:, 2);
ds = MDH_Table(:, 3);
thetas = MDH_Table(:, 4);

fprintf('\n\n\t\t\t\t\t <strong>MDH Table</strong>\n\n')
disp(table(indices, alphas, as, ds, thetas, 'VariableNames', [" ", "alpha(i-1)", "a(i-1)", "d(i)", "theta(i)"]))

