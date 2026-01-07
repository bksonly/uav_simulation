clear 
dt=0.005;
T=30;
t0=0:dt:(1-dt);
t1=0:dt:T-1;
t=0:dt:T;



k=2;
x = [0*t0, 5 * sin(t1/k)];
y = [0*t0, 5 * cos(t1/k) .* sin(t1/k)];
z=[0*t0, -0.1*t1];
psi_d=0.01*t;

% x = [1*t];
% x = [0*t0, 0*t1+1];
% y = [0*t0, 0*t1];
% z=[0*t0, 0*t1];
% psi_d=0*t;


pd=[t(1:length(x))' x' y' z']; 

for i=1:length(t)-1
    vx(i)=(x(i+1)-x(i))/dt;
    vy(i)=(y(i+1)-y(i))/dt;
    vz(i)=(z(i+1)-z(i))/dt;
end
vd=[t(1:length(vx))' vx' vy' vz'];

for i=1:length(t)-2
    ax(i)=(vx(i+1)-vx(i))/dt;
    ay(i)=(vy(i+1)-vy(i))/dt;
    az(i)=(vz(i+1)-vz(i))/dt;
end
ad=[t(1:length(ax))' ax' ay' az'];

psi_d=[t(1:length(psi_d))' psi_d'];

% J=diag([0.003,0.003,0.005]);
% m=4.34;
% J=diag([0.082,0.0845,0.1377]);

m=0.8771;
J=diag([0.0031 0.0032 0.0045]);


A=[zeros(3,3),eye(3);zeros(3,6)];
B=[zeros(3,3);eye(3)];
Q=diag([100 100 30 0.1 0.1 0.1]);
R=diag([1 1 1])/100;
sys=ss(A,B,eye(6),zeros(6,3));
K=lqr(sys,Q,R);

Qx=diag([10 10 10 3 3 3]);
Rx=diag([1 1 1])/10;
K1=lqr(sys,Qx,Rx);
kx=m*K1(1,1);%位置环误差的线性系统与姿态环一模一样！
kv=m*K1(1,4);

kt=2.03e-8;%转速平方*kt=力
tau=0.01;%电机时延

ctf=8e-3;
d=0.125;
f1234tofm=[1 1 1 1;
    0 -d 0 d;
    d 0 -d 0;
    -ctf ctf -ctf ctf];
fmtof1234=inv(f1234tofm);

sim("double_sim_motor_px4.slx")


simT=length(outx(:,1));

%% x
figure 
subplot(5,1,1)
plot(t(1:simT),x(1:simT),t(1:simT),outx(:,1))
legend('Desired','Actual');
xlabel('t/s'); ylabel('x/m','Interpreter','tex');
grid on

subplot(5,1,2)
plot(t(1:simT),vx(1:simT),t(1:simT),outv(:,1))
legend('Desired','Actual');
xlabel('t/s'); ylabel('v_x/(m/s)','Interpreter','tex');
grid on

subplot(5,1,3)
plot(t(1:simT),outda(:,2),t(1:simT),outa(:,2))
legend('Desired','Actual');
xlabel('t/s'); ylabel('\theta/rad','Interpreter','tex');
grid on

subplot(5,1,4)
plot(t(1:simT),outwd(:,2),t(1:simT),outw(:,2))
legend('Desired','Actual');
xlabel('t/s'); ylabel('\omega_2/(rad/s)','Interpreter','tex');
grid on

subplot(5,1,5)
plot(t(1:simT),fM(:,3))
xlabel('t/s'); ylabel('M_2/Nm','Interpreter','tex');
grid on

set(gcf, 'Position', [100, 100, 400, 450]);  % gcf 获取当前图窗句柄
print('-depsc', 'C:\Users\zzy\毕业论文\matlab_figures\px4_1'); % '-dpng' 表示输出PNG格式，'-r300' 表示分辨率为300 dpi

ph=450;
pw=400;
%% xyz
figure
subplot(3,1,1)
plot(t(1:simT),x(1:simT),t(1:simT),outx(:,1))
legend('Desired','Actual');
xlabel('t/s'); ylabel('x/m');

subplot(3,1,2)
plot(t(1:simT),y(1:simT),t(1:simT),outx(:,2))
legend('Desired','Actual');
xlabel('t/s'); ylabel('y/m');

subplot(3,1,3)
plot(t(1:simT),z(1:simT),t(1:simT),outx(:,3))
legend('Desired','Actual');
xlabel('t/s'); ylabel('z/m');

set(gcf, 'Position', [100, 100, pw, ph]);  
print('-depsc', 'C:\Users\zzy\毕业论文\matlab_figures\px4_x'); 
%% 角度
figure
subplot(3,1,1)
plot(t(1:simT),outda(:,3),t(1:simT),outa(:,1))
legend('Desired','Actual');
xlabel('t/s'); ylabel('\phi/rad','Interpreter','tex');

subplot(3,1,2)
plot(t(1:simT),outda(:,2),t(1:simT),outa(:,2))
legend('Desired','Actual');
xlabel('t/s'); ylabel('\theta/rad','Interpreter','tex');

subplot(3,1,3)
plot(t(1:simT),psi_d(1:simT,2),t(1:simT),outa(:,3))
legend('Desired','Actual');
xlabel('t/s'); ylabel('\psi/rad','Interpreter','tex');

set(gcf, 'Position', [100, 100, pw, ph]); 
print('-depsc', 'C:\Users\zzy\毕业论文\matlab_figures\px4_angle'); 
%% fM
figure 
subplot(4,1,1)
plot(t(1:simT),fM(:,1))
legend('f');
xlabel('t/s'); ylabel('f/N');

subplot(4,1,2)
plot(t(1:simT),fM(:,2))
legend('M1');
xlabel('t/s'); ylabel('M_1/Nm','Interpreter','tex');

subplot(4,1,3)
plot(t(1:simT),fM(:,3))
legend('M2');
xlabel('t/s'); ylabel('M_2/Nm','Interpreter','tex');

subplot(4,1,4)
plot(t(1:simT),fM(:,4))
legend('M3');
xlabel('t/s'); ylabel('M_3/Nm','Interpreter','tex');
set(gcf, 'Position', [100, 100, pw, ph]);   
print('-depsc', 'C:\Users\zzy\毕业论文\matlab_figures\px4_fM'); 

% %% f1234
% figure 
% subplot(4,1,1)
% plot(t(1:simT),responsef(:,1))
% legend('f1');
% xlabel('t/s'); ylabel('f1/N');
% 
% subplot(4,1,2)
% plot(t(1:simT),responsef(:,2))
% legend('f2');
% xlabel('t/s'); ylabel('f2/N');
% 
% subplot(4,1,3)
% plot(t(1:simT),responsef(:,3))
% legend('f3');
% xlabel('t/s'); ylabel('f3/N');
% 
% subplot(4,1,4)
% plot(t(1:simT),responsef(:,4))
% legend('f4');
% xlabel('t/s'); ylabel('f4/N');
% 

%% 3d
figure
plot3(x,y,z);
hold on
plot3(outx(:,1),outx(:,2),outx(:,3));
xlabel('x/m'); ylabel('y/m');zlabel('z/m')
legend('Desired','Actual','location','northwest')
axis equal;
set(gcf, 'Position', [100, 100, 450, 300]);  
print('-depsc', 'C:\Users\zzy\毕业论文\matlab_figures\px4_3d');
%% e
sum_e=0;
sum_angle=0;
sum_J = 0;
for i=1:simT
    Ji = ([outx(i,:)-[x(i),y(i),z(i)],outv(i,:)-[vx(i),vy(i),vz(i)]])*Qx*([outx(i,:)-[x(i),y(i),z(i)],outv(i,:)-[vx(i),vy(i),vz(i)]])'+Rx(1,1)*fM(i,1)^2;
    ei=sqrt((outx(i,1)-x(i))^2+(outx(i,2)-y(i))^2+(outx(i,3)-z(i))^2);
    e_angle_i=outa(i,1)+outa(i,2)+outa(i,3)-(outda(i,1)+outda(i,2)+outda(i,3));
    sum_J=sum_J+Ji;
    sum_e=sum_e+ei;
    sum_angle=sum_angle+abs(e_angle_i);
%     e(i)=ei;
%     e_psi(i)=e_angle_i;
end

averege_e=sum_e/simT;
averege_e_angle=sum_angle/simT;


% figure
% plot(t(1:length(e_psi)),e)
% hold on
% plot(t(1:length(e_psi)),e_psi)
% legend("e-d","e-psi")
% error_text = sprintf('平均距离跟踪误差: %.4f,平均偏航角跟踪误差: %.4f', averege_e, averege_e_psi);
% text(simT*0.2, averege_e, error_text, 'FontSize', 12, 'Color', 'black');
% xlabel('t/s');
% ylabel('e-d /m  e-psi /rad');