function link = DrawLink(Link, T, color)
% Description: this function draws a link in 3D space.
% 
% Inputs:
% Link = link object
% T = link's associated joint transformation matrix
% color = link color
% 
% Outputs:
% 
% link = structure with each figure element for the link

light_orange = [0.8902    0.4230    0.1692];

if nargin < 3
    color = light_orange;
end

% Retrieve the Dimensions
D = Link.Diameter * 1e-3;
r = D*1.2/2;
L = Link.Length;

% Retrieve the Pose
X = trans2pose(T);
x0 = X(1);
y0 = X(2);
z0 = X(3);

R = T(1:3, 1:3);

% Create the Cylinder along the z-axis
n_points = 50;      % n° of points to create a smooth cylinder
[zz, yy, xx] = cylinder(r, n_points);
xx = xx * L;        % scale the x-dimension for its length

% % Set the origin of the Cylinder
% xx = xx + x0;
% yy = yy + y0;
% zz = zz + z0;

% Rotation of the points
all_points = [xx(:)'; yy(:)'; zz(:)'];

rotated_points = R * all_points;
xx = x0+reshape(rotated_points(1,:), size(xx));
yy = y0+reshape(rotated_points(2,:), size(yy));
zz = z0+reshape(rotated_points(3,:), size(zz));

% Plot the Cylinder
core = surf(xx, yy, zz, 'EdgeColor', 'none', 'HandleVisibility','off', 'FaceColor', color);

% Compute the End Caps Coordinates
endcap_top_x = xx(1, :);
endcap_top_y = yy(1, :);
endcap_top_z = zz(1, :);

endcap_bot_x = xx(2, :);
endcap_bot_y = yy(2, :);
endcap_bot_z = zz(2, :);

% Draw the End Caps
top = fill3(endcap_top_x, endcap_top_y, endcap_top_z, color, 'EdgeColor','none', 'HandleVisibility','off');
bot = fill3(endcap_bot_x, endcap_bot_y, endcap_bot_z, color, 'EdgeColor','none', 'HandleVisibility','off');

link = struct('name', 'Link');
link.core = core;
link.top = top;
link.bot = bot;

end