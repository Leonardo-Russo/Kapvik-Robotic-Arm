function show_table(MDH_Table)
% Description: this function is the display function for the MDH table.

indices = (1:4)';
alphas = MDH_Table(:, 1);
as = MDH_Table(:, 2);
ds = MDH_Table(:, 3);
thetas = MDH_Table(:, 4);

fprintf('\n\n\t\t\t\t\t <strong>MDH Table</strong>\n\n')
disp(table(indices, alphas, double(as), ds, thetas, 'VariableNames', ["i", "alpha(i-1)", "a(i-1)", "d(i)", "theta(i)"]))


end