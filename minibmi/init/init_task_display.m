function handles = init_task_display(handles)

handles.task_display = figure(); 
handles.cursor_pos = [0, 0];
handles.target_pos = [0, 0];
handles.cursor_color = [0 1 1];
handles.target_color = [0 1 0];

%Check if target / cursor are set
if ~isfield(handles,'target_radius')
    fprintf('WARNING: no target radius')
end

if ~isfield(handles,'cursor_radius')
    fprintf('WARNING: no cursor radius')
end

% Init Display
handles.ax = gca(handles.task_display); 
hold(handles.ax, 'on')

%Cursor and Target Objects
handles.cursor = plot(handles.ax, handles.cursor_pos(1), handles.cursor_pos(2),'o',...
    'MarkerSize', handles.cursor_radius,'MarkerFaceColor',handles.cursor_color,...
    'MarkerEdgeColor', handles.cursor_color);

handles.target = plot(handles.ax, handles.target_pos(1), handles.target_pos(2),'o',...
    'MarkerSize', handles.target_radius,'MarkerFaceColor',handles.target_color, ...
    'MarkerEdgeColor', handles.target_color);

blit_display(handles.ax)