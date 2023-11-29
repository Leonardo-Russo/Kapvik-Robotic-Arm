function env = show_env(L, w, h)
% Description: this function serves to plot the static elements in the
% enviroment.

box = plot3DBox(L, w, h);           % Create the Box
hold on
terrain = show_plane([1 0 0], [0 1 0], [0 0 0]);    % Create the Terrain

% Plot Options
xlabel('$X_S$', 'fontsize', 12,'Interpreter', 'latex')
ylabel('$Y_S$', 'fontsize', 12,'Interpreter', 'latex')
zlabel('$Z_S$', 'fontsize', 12,'Interpreter', 'latex')

axis equal
grid on
view([140, 20]);
hold on

% Return Enviroment
env = struct('name', "Enviroment");
env.box = box;
env.terrain = terrain;

end