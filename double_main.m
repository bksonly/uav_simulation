clear 
dt=0.005;
T=30;
t0=0:dt:(1-dt);
t1=0:dt:T-1;
t=0:dt:T;

% x=0*t;
% y=0*t;
% z=0*t;
% x=-1./(1+exp(t-5))+1;   
% y=log(0.2*t+1);
% z=-min(0.1*t.*t,0.1*t);
% psi_d=0.1*t;

% x=cos(min(t.*t.*t/12,0.25*t.*t-0.25*t+1/12))-1;
% y=sin(min(t.*t.*t/12,0.25*t.*t-0.25*t+1/12));
% z=0*t-0.1;
% psi_d=[[0:dt:1].*[0:dt:1]/4,0.5*[1.005:dt:T]-0.25];

% x=[[0:dt:1].*[0:dt:1],2*[1.005:dt:20]-1];
% y=-x/2;
% z=0.1*t;
% psi_d=[[0:dt:1].*[0:dt:1]/4,0.5*[1.005:dt:20]-0.25];

k=2;
x = [0*t0, 3 * sin(t1/k)];
y = [0*t0, 3 * cos(t1/k) .* sin(t1/k)];
z=[0*t0, -0.1*t1];
psi_d=0.01*t;

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

m=0.8;
J=diag([0.0024 0.0025 0.0041]);


A=[zeros(3,3),eye(3);zeros(3,6)];
B=[zeros(3,3);eye(3)];
Q=diag([3 3 3 1 1 1]);
R=diag([1 1 1])/10;
sys=ss(A,B,eye(6),zeros(6,3));
K=lqr(sys,Q,R);

kx=m*K(1,1);%位置环误差的线性系统与姿态环一模一样！
kv=m*K(1,4);

kt=2.03e-8;%转速平方*kt=力
tau=0.01;%电机时延

ctf=8e-3;
d=0.125;
f1234tofm=[1 1 1 1;
    0 -d 0 d;
    d 0 -d 0;
    -ctf ctf -ctf ctf];
fmtof1234=inv(f1234tofm);


% sim("double_sim.slx")
sim("double_sim_motor.slx")

simT=length(outx(:,1));

%% 
figure
subplot(3,1,1)
plot(t(1:simT),x(1:simT),t(1:simT),outx(:,1))
legend('desired','response');
xlabel('t/s'); ylabel('x/m');

subplot(3,1,2)
plot(t(1:simT),y(1:simT),t(1:simT),outx(:,2))
legend('desired','response');
xlabel('t/s'); ylabel('y/m');

subplot(3,1,3)
plot(t(1:simT),z(1:simT),t(1:simT),outx(:,3))
legend('desired','response');
xlabel('t/s'); ylabel('z/m');

%%
figure
subplot(3,1,1)
plot(t(1:simT),outda(:,3),t(1:simT),outa(:,1))
legend('desired','response');
xlabel('t/s'); ylabel('phi/Rad');

subplot(3,1,2)
plot(t(1:simT),outda(:,2),t(1:simT),outa(:,2))
legend('desired','response');
xlabel('t/s'); ylabel('theta/Rad');

subplot(3,1,3)
plot(t(1:simT),psi_d(1:simT,2),t(1:simT),outa(:,3))
legend('desired','response');
xlabel('t/s'); ylabel('psi/Rad');
%%
figure 
subplot(4,1,1)
plot(t(1:simT),responsef(:,1))
legend('f1');
xlabel('t/s'); ylabel('f1/N');

subplot(4,1,2)
plot(t(1:simT),responsef(:,2))
legend('f2');
xlabel('t/s'); ylabel('f2/N');

subplot(4,1,3)
plot(t(1:simT),responsef(:,3))
legend('f3');
xlabel('t/s'); ylabel('f3/N');

subplot(4,1,4)
plot(t(1:simT),responsef(:,4))
legend('f4');
xlabel('t/s'); ylabel('f4/N');

figure
plot3(x,y,z);
hold on
plot3(outx(:,1),outx(:,2),outx(:,3));
xlabel('x/m'); ylabel('y/m');zlabel('z/m')
legend('desired','response')
axis equal;

sum_e=0;
sum_psi=0;
for i=1:simT
    ei=sqrt((outx(i,1)-x(i))^2+(outx(i,2)-y(i))^2+(outx(i,3)-z(i))^2);
    e_psi_i=outa(i,3)-psi_d(i,2);
    sum_e=sum_e+ei;
    sum_psi=sum_psi+abs(e_psi_i);
    e(i)=ei;
    e_psi(i)=e_psi_i;
end

averege_e=sum_e/simT;
averege_e_psi=sum_psi/simT;
%%

figure
plot(t(1:length(e_psi)),e)
hold on
plot(t(1:length(e_psi)),e_psi)
legend("e-d","e-psi")
error_text = sprintf('平均距离跟踪误差: %.4f,平均偏航角跟踪误差: %.4f', averege_e, averege_e_psi);
text(simT*0.2, averege_e, error_text, 'FontSize', 12, 'Color', 'black');
xlabel('t/s');
ylabel('e-d /m  e-psi /rad');