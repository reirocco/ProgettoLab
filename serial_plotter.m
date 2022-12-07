%%
clc,clear
%% serial port list
serialportlist("available")
%% open serial port
device = serialport("COM3",115200);
%% start plotting
readline(device)
readline(device)
figure
linewidth = 1.5;
h = animatedline('Color',[1 0 0],'LineWidth',linewidth, 'MaximumNumPoints',500);% 
g = animatedline('Color',[1 0 1] ,'LineWidth',linewidth, 'MaximumNumPoints',500);% yellow realvref
y = animatedline('Color',[0 1 1] ,'LineWidth',linewidth, 'MaximumNumPoints',500);%cyan voltmedia
% h = animatedline('Color',[1 0 0],'LineWidth',1);% 
% g = animatedline('Color',[1 0 1] ,'LineWidth',1);% yellow realvref
% y = animatedline('Color',[0 1 1] ,'LineWidth',1);%cyan voltmedia
x = 0;
while 1
    raw = readline(device);
    formattedRaw = double(sprintf("%f",raw))

    line = readline(device);
    realVref = sprintf("%f",line)

    line = readline(device);
    voltMedia = sprintf("%f",line)

   % [realVref,voltMedia] = split(formatted_line,",");
    
    
 
    %disp(formatted_line)
    addpoints(g,double(x),double(realVref))
    addpoints(y,double(x),double(voltMedia))
    addpoints(h,double(x),double(formattedRaw))
    drawnow
    x = x + 1;
end


%%
clear device