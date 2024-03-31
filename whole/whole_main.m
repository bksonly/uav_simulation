clear
dt=0.005;
t=0:dt:8;

% x=0*t;
% y=0*t;
% z=0*t;
x=-1./(1+exp(t-5))+1;   
y=log(0.2*t+1);
z=-min(0.1*t.*t,0.1*t);
psi_d=0.1*t';

% x=cos(min(t.*t.*t/12,0.25*t.*t-0.25*t+1/12))-1;
% y=sin(min(t.*t.*t/12,0.25*t.*t-0.25*t+1/12));
% z=0*t;
% psi_d=[[0:dt:1].*[0:dt:1]/4,0.5*[1.005:dt:8]-0.25]';

pd=[x' y' z']; 

for i=1:length(t)-1
    vx(i)=(x(i+1)-x(i))/dt;
    vy(i)=(y(i+1)-y(i))/dt;
    vz(i)=(z(i+1)-z(i))/dt;
end
vd=[vx' vy' vz'];

for i=1:length(t)-2
    ax(i)=(vx(i+1)-vx(i))/dt;
    ay(i)=(vy(i+1)-vy(i))/dt;
    az(i)=(vz(i+1)-vz(i))/dt;
end
ad=[ax' ay' az'];



% J=diag([0.003,0.003,0.005]);
m=4.34;
J=diag([0.082,0.0845,0.1377]);
kx=16;
kv=5.6;

A=[zeros(3,3),eye(3);zeros(3,6)];
B=[zeros(3,3);eye(3)];
Q=diag([3 3 3 1 1 1]);
R=diag([1 1 1])/10;
sys=ss(A,B,eye(6),zeros(6,3));
K=lqr(sys,Q,R);

sim("whole.slx")


figure
subplot(2,3,1)
plot(t(1:length(outx(:,1))),x(1:length(outx(:,1))),t(1:length(outx(:,1))),outx(:,1))
legend('desired','out');
xlabel('t'); ylabel('x m/s');

subplot(2,3,2)
plot(t(1:length(outx(:,2))),y(1:length(outx(:,2))),t(1:length(outx(:,2))),outx(:,2))
legend('desired','out');
xlabel('t'); ylabel('y m/s');

subplot(2,3,3)
plot(t(1:length(outx(:,3))),z(1:length(outx(:,3))),t(1:length(outx(:,3))),outx(:,3))
legend('desired','out');
xlabel('t'); ylabel('z m/s');

subplot(2,3,4)
plot(t(1:length(outa(:,3))),psi_d(1:length(outa(:,3))),t(1:length(outa(:,3))),outa(:,3))
legend('desired','out');
xlabel('t'); ylabel('psi/Rad');

subplot(2,3,5)
plot(outx(:,1),outx(:,2));
hold on
plot(x,y);
xlabel('x'); ylabel('y');
