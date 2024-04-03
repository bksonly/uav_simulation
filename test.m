% x=0.3;
% y=0.1;
% z=0.2;
% RdT=angle2dcm(z,y,x,"ZYX");
% 
% Rd=RdT';
% R=diag([1 1 1]);
% eR=vee(RdT*R-R'*Rd)
% e=[1;1;1]-diag(RdT*R)
% e_india=cross([1;0;0],RdT*R(:,1))+cross([0;1;0],RdT*R(:,2))+cross([0;0;1],RdT*R(:,3))

% Rd1=[0.9951    0.0993   -0.0043;
%     0.0990   -0.9943   -0.0392;
%    -0.0081    0.0386   -0.9992];
% Rd2=[0.9950   -0.0998   -0.0044
%     0.0995    0.9942   -0.0406
%     0.0084    0.0399    0.9992];
% % 对Rd直接差分
% dRd=(Rd2-Rd1)/0.005;
% 
% lR=inv(Rd1)*dRd;
% rR=inv(Rd2)*dRd;
% 
% % 李代数
% % ln_matrix(Rd1)%发散 不知为何
% 
% % 欧拉角的差 奇异
% % [z1,y1,x1]=dcm2angle(Rd1,"ZYX");
% % [z2,y2,x2]=dcm2angle(Rd2,"ZYX");
% % ([z2,y2,x2]-[z1,y1,x1])/0.005
% 
% 
% %差的欧拉角
% [w1,w2,w3]=dcm2angle(Rd1'*Rd2,"ZYX")
% % Rd2*Rd1'
dt=0.005;
T=30;
t0=0:dt:(1-dt);
t1=0:dt:T-1;
t=0:dt:T;

k=2;
x = [0*t0, 3 * sin(t1/k)];
y = [0*t0, 3 * cos(t1/k) .* sin(t1/k)];
z=[0*t0, -0.1*t1];
psi_d=0.01*t;

% 绘制轨迹
plot3(x, y,z);
grid on;
xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴');

axis equal; % 保持轴的比例一致

