function terrain = show_plane(u, v, p, size, color)
% Description: this function draws the plane defined by the input vectors
% u and v at point P.

if nargin < 4
    size = 1;
end

if nargin < 5
    color = [219, 219, 219];
    color = color / norm(color);
end

[P,Q] = meshgrid(-size:size);     % provide a gridwork

% Compute the corresponding cartesian coordinates using the two vectors in w
x1 = p(1);
y1 = p(2);
z1 = p(3);

X = x1 + u(1)*P + v(1)*Q;
Y = y1 + u(2)*P + v(2)*Q; 
Z = z1 + u(3)*P + v(3)*Q;

terrain = surf(X,Y,Z, 'FaceColor', color, 'FaceAlpha', 0.25, 'EdgeAlpha', 0.25, 'HandleVisibility','off');
% hold on
% %%% Mars terrain
% MarsTerrain = imread('mars.jpg');
% %%% Plot Martian soil
% warp(X,Y,Z,MarsTerrain);

end