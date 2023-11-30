function update_link(link, T, Link)

% Retrieve the Dimensions
D = Link.Diameter * 1e-3;
r = D*1.2/2;
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

% Rotation of the points
all_points = [xx(:)'; yy(:)'; zz(:)'];
R = R3(-yaw,"deg")*R2(-pitch,"deg")*R1(-roll,"deg");
rotated_points = R * all_points;
xx = x0+reshape(rotated_points(1,:), size(xx));
yy = y0+reshape(rotated_points(2,:), size(yy));
zz = z0+reshape(rotated_points(3,:), size(zz));

% Update the Cylinder
set(link.core, 'XData', xx, 'YData', yy, 'ZData', zz);

% Update the End Caps
set(link.top, 'XData', xx(1, :), 'YData', yy(1, :), 'ZData', zz(1, :));
set(link.bot, 'XData', xx(2, :), 'YData', yy(2, :), 'ZData', zz(2, :));

end
