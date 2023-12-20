function env = show_env(L, w, h)
% Description: this function serves to plot the static elements in the
% enviroment.

roverBox = plot3DBox(L, w, h);           % Create the rover box
hold on
terrain = show_plane([1 0 0], [0 1 0], [0 0 0]);    % Create the Terrain

% Define supportBox dimensions
ls = 0.05;
hs = 0.09;
ws = 0.025;
% Define supportBox position
position = [0.06, -0.6, 0.3];
supportBox = plot3DBox(ls, ws, hs, [0.7, 0.7, 0.7]);
vertices = get(supportBox, 'Vertices');
set(supportBox, 'Vertices', bsxfun(@plus, vertices, position));

% Define deliveryBox dimensions
ld = 0.1;
hd = 0.07;
wd = 0.1;
% Define supportBox position
position = [-0.3, -0.05, 0.23];
deliveryBox = plotDeliveryBox(ld, wd, hd, [0.9, 0.9, 0.9]);
vertices = get(deliveryBox, 'Vertices');
set(deliveryBox, 'Vertices', bsxfun(@plus, vertices, position));

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
env.box = roverBox;
env.suppBox = supportBox;
env.deliBox = deliveryBox;
env.terrain = terrain;

end