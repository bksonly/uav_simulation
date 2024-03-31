clear
dt=0.01;
t=0:dt:11;

x=0*t;
y=0*t;
z=0*t;
x=-1./(1+exp(t-5))+1;
y=0.2*sin(0.5*t);
z=min(0.1*t.*t,0.1*t);
dtheta=[x;y;z]';
RdT=angle2dcm(z,y,x,"ZYX");
for i=1:size(RdT,3)
    Rd(:,:,i)=RdT(:,:,i)';
end
for i=1:length(t)-1
    tmp=inv(Rd(:,:,i))*(Rd(:,:,i+1)-Rd(:,:,i))/dt;
    wd(i,:)=vee(tmp);
end
for i=1:length(t)-2
    ad(i,:)=(wd(i+1,:)-wd(i,:))/dt;
end
% J=diag([0.003,0.003,0.005]);
J=diag([0.082,0.0845,0.1377]);

A=[zeros(3,3),eye(3);zeros(3,6)];
B=[zeros(3,3);eye(3)];
Q=diag([ 10 10 3 3 3 1]);
R=diag([1 1 1])/10;
% rank(ctrb(A,B));
sys=ss(A,B,eye(6),zeros(6,3));
K=lqr(sys,Q,R);
% sys1=ss(A-B*K,B,eye(6),zeros(6,3));
% step(sys1)

sim("atti.slx")



subplot(1,3,1)
plot(t(1:length(outa(:,1))),x(1:length(outa(:,1))),t(1:length(outa(:,1))),outa(:,1))
legend('out','desired');
xlabel('t'); ylabel('phi/Rad');

subplot(1,3,2)
plot(t(1:length(outa(:,2))),y(1:length(outa(:,2))),t(1:length(outa(:,2))),outa(:,2))
legend('out','desired');
xlabel('t'); ylabel('theta/Rad');

subplot(1,3,3)
plot(t(1:length(outa(:,3))),z(1:length(outa(:,3))),t(1:length(outa(:,3))),outa(:,3))
legend('out','desired');
xlabel('t'); ylabel('psi/Rad');