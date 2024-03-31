function [sys,x0,str,ts,simStateCompliance] = whole_controller_2usin(t,x,u,flag,dt,pd,vd,ad,psi_d,m,J,kx,kv,K)

%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(dt);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u,dt,pd,vd,ad,psi_d,m,J,kx,kv,K);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,dt,pd,vd,ad,psi_d,m,J,kx,kv,K);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(dt)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 15;%Rd wd dwd
sizes.NumOutputs     = 5;%f M
sizes.NumInputs      = 18;%x v R w
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
 x0  =zeros([15 1]);
%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%

ts  = [dt 0];


% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,dt,pd,vd,ad,psi_d,m,J,kx,kv,K)
g=9.8;
i=fix(t/dt)+1;
ipd=pd(i,:)';
ivd=vd(i,:)';
iad=ad(i,:)';
ipsi_d=psi_d(i);

p=reshape(u(1:3),[3,1]);
v=reshape(u(4:6),[3,1]);

ex=ipd-p;
ev=ivd-v;

pre_Rd=reshape(x(1:9),[3,3]);
pre_wd=reshape(x(10:12),[3,1]);
f_ideal=kx*ex+kv*ev-m*g*[0;0;1]+m*iad;%不考虑升力只能向下的理想升力

b1d=[cos(ipsi_d);sin(ipsi_d);0];
b3d=-f_ideal/norm(f_ideal);%z轴和升力方向相反
b2d=cross(b3d,b1d)/norm(cross(b3d,b1d));
Rd=[cross(b2d,b3d),b2d,b3d];

% if t==1
%     disp(pre_Rd)
%     disp(Rd);
% end

if trace(pre_Rd)==0
    wd=[0;0;0];
else
    Wd=inv(Rd)*(Rd-pre_Rd)/dt;
    wd=vee(Wd);
end

if norm(wd)==0
    dwd=[0;0;0];
else
    dwd=(wd-pre_wd)/dt;
end

% wd=reshape(u(16:18),[3,1]);
% dwd=[0;0;0];

% if norm(wd)>0.1
%     wd=wd/norm(wd)/10;
% end
% 
% if norm(dwd)>0.1
%     dwd=dwd/norm(dwd)/10;
% end

sys = [reshape(Rd,[9,1]);wd;dwd];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,dt,pd,vd,ad,psi_d,m,J,kx,kv,K)
g=9.8;
%参数处理
i=fix(t/dt)+1;
ipd=pd(i,:)';
ivd=vd(i,:)';
iad=ad(i,:)';


%反馈信息处理
p=reshape(u(1:3),[3,1]);
v=reshape(u(4:6),[3,1]);
R=reshape(u(7:15),[3,3]);
w=reshape(u(16:18),[3,1]);

%内部状态读取
Rd=reshape(x(1:9),[3,3]);
wd=reshape(x(10:12),[3,1]);
dwd=reshape(x(13:15),[3,1]);

%计算f
ex=ipd-p;
ev=ivd-v;
f_ideal=kx*ex+kv*ev-m*g*[0;0;1]+m*iad;%不考虑升力只能向下的理想升力
f=f_ideal'*(R*[0;0;1]);%在body的z轴上投影，还是负的

%计算M
Rr=Rd'*R;
eR=0.5*vee(Rr-Rr');
ew=w-R'*Rd*wd;
hat_ew=hat(ew);

tmp1=Rr*hat_ew*hat(w)-hat(wd)*Rr*hat_ew- hat(dwd)*Rr;
tmp=tmp1-tmp1';

RB=[Rr(2,2)+Rr(3,3)       -Rr(2,1)          -Rr(3,1);
     -Rr(1,2)         Rr(1,1)+Rr(3,3)       -Rr(3,2);
     -Rr(1,3)           -Rr(2,3)          Rr(1,1)+Rr(2,2)];

e=vee(Rr-Rr');
de=vee(Rr*hat_ew+hat_ew*Rr');

if rank(RB)==3
    B=RB*inv(J);
    M=inv(B)*(-vee(tmp)-K*[e;de])+cross(w,J*w);
    unfullrank=0;
else
    M=-8.81*eR-2.54*ew+cross(w,J*w)-J*(hat(w)*Rr'*wd-Rr'*dwd);
    unfullrank=1;
end
% M(1:3)=-8.81*eR-2.54*ew+cross(w,J*w)-J*(hat(w)*Rr'*wd-Rr'*dwd);
% unfullrank=1;
% th=1;
% M(1)=deadzone(-th,M(1),th);
% M(2)=deadzone(-th,M(2),th);
% M(3)=deadzone(-th,M(3),th);

sys =[f;M;unfullrank];

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
