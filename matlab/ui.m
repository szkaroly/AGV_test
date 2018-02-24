function [ velocity, position ] = ui
    % Create a figure and axes
    f = figure(1);
  
   % Create slider
    sld_1 = uicontrol('Style', 'slider',...
        'Min',1,'Max',50,'Value',11,...
        'Position', [200 200 120 20],...
        'Callback', @slider_1); 

    % Add a text uicontrol to label the slider.
    txt_1 = uicontrol('Style','text',...
        'Position',[200 225 120 20],...
        'String','Speed');    
    
   % Create slider
    sld_2 = uicontrol('Style', 'slider',...
        'Min',1,'Max',50,'Value',41,...
        'Position', [200 150 120 20],...
        'Callback', @slider_2); 
    
    % Add a text uicontrol to label the slider.
    txt_2 = uicontrol('Style','text',...
        'Position',[200 175 120 20],...
        'String','Steering');

   % Create slider
    sld_3 = uicontrol('Style', 'slider',...
        'Min',1,'Max',50,'Value',41,...
        'Position', [200 100 120 20],...
        'Callback', @slider_3); 
    
    % Add a text uicontrol to label the slider.
    txt_3 = uicontrol('Style','text',...
        'Position',[200 125 120 20],...
        'String','Fork');
    
       % Create push button
    btn = uicontrol('Style', 'pushbutton', 'String', 'Stop',...
        'Position', [20 20 50 20],...
        'Callback', @stop); 
    
    % Make figure visble after adding all components
%    f.Visible = 'on';

% Version A
    function slider_1(source,event)
        disp(source.Value);
%        velocity = source.Value;
%        position = 2*source.Value;
    end

    function slider_2(source,event)
        disp(source.Value);
    end

    function slider_3(source,event)
        disp(source.Value);
    end

    function stop(source,event)
        disp('UI stopped');
    end


% Version B
%    x=get(sld_1,'Value');
%    disp(x);

    disp(sld_1.Value);
    disp(btn.Value);

end