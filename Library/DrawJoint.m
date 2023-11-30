function joint = DrawJoint(Di, Do, L, T)
% Description: Draws a revolute joint consisting of two cylinders in 3D
% space.
    
blue = [0 0.4470 0.7410];
orange = [0.8500, 0.3250, 0.0980];

% Create the inner cylinder (fixed part) and its end caps
[~, ~, ~, caps_inner] = create_cylinder(Di, L, T); % No rotation

% Create the outer cylinder (rotating part) and its end caps
[xx_outer, yy_outer, zz_outer, caps_outer] = create_cylinder(Do, L, T);

% Plot the Outer Cylinder
core = surf(xx_outer, yy_outer, zz_outer, 'FaceColor', orange, 'EdgeColor', 'none', 'HandleVisibility','off');

% Plot the End Caps for the Outer Cylinder (hollow)
outcaps = draw_caps(caps_outer, orange, caps_inner);

% Plot the End Caps for the Inner Cylinder (solid)
incaps = draw_caps(caps_inner, blue);

joint = struct('name', "joint");
joint.core = core;
joint.outcaps = outcaps;
joint.incaps = incaps;
joint.Di = Di;
joint.Do = Do;
joint.L = L;

end

function caps = draw_caps(caps, color, inner_caps)

    if nargin < 3

        % Draw solid end caps
        topcap = fill3(caps{1, 1}, caps{1, 2}, caps{1, 3}, color, 'EdgeColor', 'none', 'HandleVisibility','off');
        botcap = fill3(caps{2, 1}, caps{2, 2}, caps{2, 3}, color, 'EdgeColor', 'none', 'HandleVisibility','off');

    else

        % Draw hollow end caps for the outer cylinder
        topcap = fill3([caps{1, 1}, fliplr(inner_caps{1, 1})], ...
              [caps{1, 2}, fliplr(inner_caps{1, 2})], ...
              [caps{1, 3}, fliplr(inner_caps{1, 3})], color, 'EdgeColor', 'none', 'HandleVisibility','off');
        
        botcap = fill3([caps{2, 1}, fliplr(inner_caps{2, 1})], ...
              [caps{2, 2}, fliplr(inner_caps{2, 2})], ...
              [caps{2, 3}, fliplr(inner_caps{2, 3})], color, 'EdgeColor', 'none', 'HandleVisibility','off');
    
    end

    caps = [topcap; botcap];

end





