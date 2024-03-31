   clc; clear; close all;
%% input（北-东-地系下）
simtime = 5;
% xd = [1  ,0,-1]';
% b1d=[1,0,0]';
t=(0:0.01:10)';
% xd=[0.4*t,0.4*cos(pi*t),0.6*cos(pi*t)];
% b1d=[cos(pi*t),sin(pi*t),0*t];
xd=[0.1    *(t.*t),0.1*t,-0.1*t];
b1d=[cos(t*pi/5) ,sin(t*pi/5),0*t];
xd=[t,xd];
b1d=[t,b1d];
%% Object properties
%  global m g J k_F
g = 9.8;
m = 0.8; % 【飞机质量】
d=0.12;
L = d; % m 【机臂长度】
k_F = 1.9e-8; %     【电机转动力系数】
% k_M = 1.5*10^(-9)*3600/(4*pi^2); %N*m/(rad/s)^2 【电机转动力矩系数】
k_m =100; % 【电机响应延时（0.05s）】
ctf=8e-3;
maxz=25000;
minz=10000;
f1234tofm=[1 1 1 1;
    0 -d 0 d;
    d 0 -d 0;
    -ctf ctf -ctf ctf];
fmtof1234=inv(f1234tofm);
Ixx = 0.003; %【转动惯量】
Iyy = 0.003; %【转动惯量】
Izz = 0.005; %【转动惯量】
J=diag([Ixx,Iyy,Izz]);



%% Run simulator
h = sim('my_exp1_hover',simtime);

%% Plot
t = h.tout;
figure;
subplot(2,2,1)
plot(t,h.position(:,1),t,h.position(:,2),t,h.position(:,3));
legend('X','Y','Z');
xlabel('t/s'); ylabel('Position/m');
grid on;
set(gca,'LooseInset',get(gca,'TightInset'));

subplot(2,2,2)
plot(t,h.velocity(:,1),t,h.velocity(:,2),t,h.velocity(:,3));
legend('V_x','V_y','V_z');
xlabel('t/s'); ylabel('Velocity (m/s)');
grid on;
set(gca,'LooseInset',get(gca,'TightInset'));

subplot(2,2,3)
plot(t,h.angle(:,1),t,h.angle(:,2),t,h.angle(:,3));
legend('\phi','\theta','\psi');
xlabel('t/s'); ylabel('Angle/rad');
grid on;
set(gca,'LooseInset',get(gca,'TightInset'));

subplot(2,2,4)
plot(t,h.wb(:,1),t,h.wb(:,2),t,h.wb(:,3));
legend('wx','wy','wz');
xlabel('t/s'); ylabel('Angle/rad/s');
grid on;
set(gca,'LooseInset',get(gca,'TightInset'));