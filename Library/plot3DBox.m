function box = plot3DBox(length, width, height, facecolor)
% Description: this function draws a box in 3D space. The reference frame
% is coherent with the choice of station frame in this case study.

if nargin < 4
    facecolor = [219, 219, 219];
    facecolor = facecolor / norm(facecolor);
end

edgecolor = [82, 82, 82];
edgecolor = edgecolor / norm(edgecolor);

% Define the vertices of the box
% The origin is on the bottom, at the center of one of the short sides
x = [-width/2, width/2, width/2, -width/2, -width/2, width/2, width/2, -width/2];
y = -[0, 0, length, length, 0, 0, length, length];
z = [0, 0, 0, 0, height, height, height, height];

% Define the vertices of each face
faces = [1, 2, 6, 5;    % Front face
         2, 3, 7, 6;    % Right face
         3, 4, 8, 7;    % Back face
         4, 1, 5, 8;    % Left face
         5, 6, 7, 8];   % Top face

% Plot the Box
box = patch('Vertices',[x',y',z'], 'Faces',faces, 'FaceColor', facecolor, 'EdgeColor', edgecolor, 'FaceAlpha', 1, 'HandleVisibility','off');

end
