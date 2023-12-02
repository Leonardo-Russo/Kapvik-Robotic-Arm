function strobEffectCallback(~, ~, lightHandle)

colors = [1 0 0; 0 0 1; 0 1 0; 1 0 1; 1 1 0; 0 1 1; 1 0.5 0; 0.5 1 0; 0 1 0.5; 1 0 0.5];
    
numFlashes = 20;    % n° of flashes

dt = 0.075;          % pause time

for i = 1 : numFlashes
    colorIndex = mod(i-1, size(colors, 1)) + 1;
    set(lightHandle, 'Visible', 'off');
    pause(dt);
    set(lightHandle, 'Color', colors(colorIndex, :), 'Visible', 'on');
    pause(dt);
end

set(lightHandle, 'Color', [1 1 1], 'Visible', 'on');

end
