function plot3DBox(length, width, height)
% Description: this function draws a box in 3D space. The reference frame
% is coherent with the choice of station frame in this case study.

% Define the vertices of the box
% The origin is on the bottom, at the center of one of the short sides
x = [-width/2, width/2, width/2, -width/2, -width/2, width/2, width/2, -width/2];
y = [0, 0, length, length, 0, 0, length, length];
z = [0, 0, 0, 0, height, height, height, height];

% Define the vertices of each face
faces = [1, 2, 6, 5;    % Front face
         2, 3, 7, 6;    % Right face
         3, 4, 8, 7;    % Back face
         4, 1, 5, 8;    % Left face
         1, 2, 3, 4;    % Bottom face
         5, 6, 7, 8];   % Top face

% Create the figure and plot the box
figure;
patch('Vertices',[x',y',z'], 'Faces',faces, 'FaceColor','blue','EdgeColor','black');
xlabel('Width');
ylabel('Length');
zlabel('Height');
axis equal;
grid on;
view(3); % Set to 3D view
end
