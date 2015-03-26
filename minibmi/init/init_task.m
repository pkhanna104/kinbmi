function [handles] = init_task(handles)
%Generically good things: 
addpath('/Users/preeykhanna/kinbmi/minibmi/')
addpath('/Users/preeykhanna/kinbmi/classes/')

%Create Task Display
handles = init_task_display(handles);

%Create Task_Data Saving


%Init Task Parameters and BMI


%Init Generator
handles.task.generator = rand(1000,1);
handles.task.rew_cnt = 0;

%Init Neural Interface
handles = init_Nexus(handles);



