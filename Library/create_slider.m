function slider = create_slider(name, minVal, maxVal, callback, slider_position, label_position, panel)
    global Q

    switch name
        case 'q1'
            value = rad2deg(Q(1));
        case 'q2'
            value = rad2deg(Q(2));
        case 'q3'
            value = rad2deg(Q(3));
        case 'q4'
            value = rad2deg(Q(4));
    end
    
    slider = uicontrol('Style', 'slider', ...
                       'Parent', panel, ...
                       'Min', minVal, 'Max', maxVal, ...
                       'Value', value, ...
                       'BackgroundColor', 'white', ...
                       'Position', slider_position, ...
                       'Callback', {@sliderCallback, callback}, ... % pass the callback function
                       'Tag', name);                % tag used to identify the slider

    label = uicontrol('Style', 'text', ...
                      'Parent', panel, ...
                      'Position', label_position, ...
                      'String', sprintf('%s: %.1f', name, value), ...
                      'fontsize', 8, ...
                      'FontWeight', 'bold', ...
                      'BackgroundColor', 'white');

    set(slider, 'UserData', label);     % store the label handle in the slider's UserData for access in the callback

    % Call the initial callback to set up the plot
    feval(callback, slider, []);
end

function sliderCallback(slider, ~, callback)
    % Get the current value of the slider
    sliderValue = get(slider, 'Value');
    
    % Get the associated label
    label = get(slider, 'UserData');
    
    % Update the displayed value on the label
    set(label, 'String', sprintf('%s: %.1f', get(slider, 'Tag'), sliderValue));
    
    % Call the original callback function
    feval(callback, slider, []);
end
