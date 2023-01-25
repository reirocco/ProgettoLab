%%
clc,clear
%% serial port list
serialportlist("available")
%% open serial port
device = serialport("COM3",115200);
%% start plotting
disp("starting...")
i = 1;
l = 1;
while l
    raw1 = readline(device);
     if raw1 == "OK"
        l = 0;
     else
    a = sprintf("%f",raw1);
   
    matrix(i,[1 2 3 4]) = [i str2double(a) 0 0];
    i = i+1;
     end
end

disp("fine calibrazione")

while 1 
    raw1 = readline(device);
    raw2 = readline(device);
    raw3 = readline(device);
    a = str2double(sprintf("%f",raw1));
    b = str2double(sprintf("%f",raw2));
    c = str2double(sprintf("%f",raw3));
    matrix(i,[1 2 3 4]) = [i a b c];
    i = i+1;
end
%% 
figure
class(matrix(1,1))
%plot(matrix(:,1),matrix(:,2));
x = matrix(:,1);
y1 = matrix(:,2);
y2 = matrix(:,3);
y3 = matrix(:,4);
ampere = (y2 - y3 ) / 0.4;
rawampere = (2.5 - y1 ) / 0.4;

plot(x,y1,x,y2,x,y3)
figure 
plot(x,ampere,x,rawampere)
