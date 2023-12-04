function S = plotScoop(L, T_12S, T_T2S, facecolor)
% Description: this function draws a box in 3D space. The reference frame
% is coherent with the choice of station frame in this case study.

if nargin < 5
    facecolor = [219, 219, 219];
    facecolor = facecolor / norm(facecolor);
end

% Retrieve the Pose
X = trans2pose(T_T2S);
r(1,1) = X(1);
r(2,1)  = X(2);
r(3,1)  = X(3);
roll = rad2deg(X(4));
pitch = rad2deg(X(5));
yaw = rad2deg(X(6));

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
S = patch('Vertices',[edges(:, 1), edges(:, 2), edges(:, 3)], 'Faces',faces, 'FaceColor', facecolor, 'EdgeColor', edgecolor, 'FaceAlpha', 0.55, 'HandleVisibility','off');
rotate(S, [0 0 1], yaw, T_T2S(1:3,4));
rotate(S, T_12S(1:3,2), pitch, T_T2S(1:3,4));
rotate(S, T_T2S(1:3,1), roll, T_T2S(1:3,4));

end
