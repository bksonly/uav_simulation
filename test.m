% T1=46.46;
% T2=46.36;
% T3=46.40;
% d=0.209;l=0.67;

% T1=58.4;
% T2=57.96;
% T3=58.37;
% d=0.17;l=0.66;

% T1=42.5;
% T2=42.75;
% T3=42.75;
% d=0.271;l=0.65;
% 
% T0=(T1+T2+T3)/50/3;
% g=9.794;m=0.8771;
% J=m*g*d^2/(16*pi^2*l)*T0^2
% 
% R=angle2dcm(10,20,15);
% q1=dcm2quat(R)
% q2=dcm2quat(R')

figure
plot3(x,y,z);
xlabel('x/m'); ylabel('y/m');zlabel('z/m')
axis equal;