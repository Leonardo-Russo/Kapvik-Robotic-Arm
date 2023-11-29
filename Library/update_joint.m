function update_joint(joint, T)

global inner_diameter outer_diameter Length

% Update Core
[xx_outer, yy_outer, zz_outer, ~] = create_cylinder(outer_diameter, Length, T);
set(joint.core, 'XData', xx_outer, 'YData', yy_outer, 'ZData', zz_outer);

% Regenerate Caps
[~, ~, ~, caps_outer] = create_cylinder(outer_diameter, Length, T);
[~, ~, ~, caps_inner] = create_cylinder(inner_diameter, Length, T);

% Update Caps
update_caps(joint.outcaps, caps_outer, caps_inner);
update_caps(joint.incaps, caps_inner);

end

function update_caps(caps_handles, new_caps, inner_caps)

if nargin < 3
    % Update Solid Caps
    set(caps_handles(1), 'XData', new_caps{1, 1}, 'YData', new_caps{1, 2}, 'ZData', new_caps{1, 3});
    set(caps_handles(2), 'XData', new_caps{2, 1}, 'YData', new_caps{2, 2}, 'ZData', new_caps{2, 3});
else
    % Update Hollow Caps
    newXDataTop = [new_caps{1, 1}, fliplr(inner_caps{1, 1})];
    newYDataTop = [new_caps{1, 2}, fliplr(inner_caps{1, 2})];
    newZDataTop = [new_caps{1, 3}, fliplr(inner_caps{1, 3})];

    newXDataBot = [new_caps{2, 1}, fliplr(inner_caps{2, 1})];
    newYDataBot = [new_caps{2, 2}, fliplr(inner_caps{2, 2})];
    newZDataBot = [new_caps{2, 3}, fliplr(inner_caps{2, 3})];

    set(caps_handles(1), 'XData', newXDataTop, 'YData', newYDataTop, 'ZData', newZDataTop);
    set(caps_handles(2), 'XData', newXDataBot, 'YData', newYDataBot, 'ZData', newZDataBot);
end

end
