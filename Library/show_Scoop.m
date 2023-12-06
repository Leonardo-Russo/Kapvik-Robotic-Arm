function S = show_Scoop(L, T_T2S, facecolor)
% Description: this function draws a box in 3D space. The reference frame
% is coherent with the choice of station frame in this case study.

if nargin < 5
    facecolor = [252, 186, 3];
    facecolor = facecolor / norm(facecolor);
end

% Retrieve the Pose
X = trans2pose(T_T2S);
r(1,1) = X(1);
r(2,1)  = X(2);
r(3,1)  = X(3);
R = T_T2S(1:3, 1:3);

edgecolor = [82, 82, 82];
edgecolor = edgecolor / norm(edgecolor);

% Define the vertices of the box
x = r(1);
y = r(2);
z = r(3);

edges = [x-L, y+L/2, z+L;
         x+L, y+L, z+L;
         x+L, y+L, z-L;
         x-L, y+L/2, z-L;
         x-L, y-L*0.8, z+L;
         x+L, y-L*0.8, z+L;
         x+L, y-L*0.8, z-L;
         x-L, y-L*0.8, z-L];


% Rotate vertices
for i = 1:size(edges,1)
    rotated = R * (edges(i,:)' - r);
    edges(i,:) = r' + rotated';
end

% Show the Edge Names
% for i = 1 : 8
%     text(edges(i, 1), edges(i, 2), edges(i, 3), string(i));
% end

% Define the vertices of each face
faces = [1, 2, 6, 5;
         2, 3, 7, 6;
         3, 4, 8, 7;
         4, 1, 5, 8;
         5, 6, 7, 8];


% Plot the Box
S = patch('Vertices', [edges(:, 1), edges(:, 2), edges(:, 3)], 'Faces', faces, 'FaceColor', facecolor, 'EdgeColor', edgecolor, 'FaceAlpha', 1, 'HandleVisibility','off');


end
