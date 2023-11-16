function Body = DrawLink(Link, T)
% Description: this function draws a link in 3D space.
% 
% Inputs:
% Link = link object
% T = link's associated joint transformation matrix

% Retrieve the Dimensions
D = Link.Diameter * 1e-3;
r = D/2;
L = Link.Length;

% Retrieve the Pose
X = trans2pose(T);
x0 = X(1);
y0 = X(2);
z0 = X(3);
roll = rad2deg(X(4));
pitch = rad2deg(X(5));
yaw = rad2deg(X(6));

% Create the Cylinder along the z-axis
n_points = 50;      % n° of points to create a smooth cylinder
[zz, yy, xx] = cylinder(r, n_points);
xx = xx * L;        % scale the x-dimension for its length

% Set the origin of the Cylinder
xx = xx + x0;
yy = yy + y0;
zz = zz + z0;

% Plot the Cylinder
core = surf(xx, yy, zz, 'EdgeColor', 'none');

% Compute the End Caps Coordinates
endcap_top_x = xx(1, :);
endcap_top_y = yy(1, :);
endcap_top_z = zz(1, :);

endcap_bot_x = xx(2, :);
endcap_bot_y = yy(2, :);
endcap_bot_z = zz(2, :);

% Draw the End Caps
top = fill3(endcap_top_x, endcap_top_y, endcap_top_z, 'y', 'EdgeColor','none');
bot = fill3(endcap_bot_x, endcap_bot_y, endcap_bot_z, 'y', 'EdgeColor','none');

Body = struct('name', "Cylinder Link");
Body.surfs = [core, top, bot];

angles = [roll, pitch, yaw];
rotate_body(Body, angles)

end