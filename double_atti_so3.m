function [sys,x0,str,ts,simStateCompliance] = double_atti(t,x,u,flag,dt,J,K)

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
    sys=mdlUpdate(t,x,u,dt);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,J,K);

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
sizes.NumOutputs     = 7;% M notfullrank
sizes.NumInputs      = 21;%R w Rd
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
 x0  =[1 0 0 0 1 0 0 0 1 0 0 0 0 0 0]';
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
function sys=mdlUpdate(t,x,u,dt)
%内部状态读取
pre_Rd=reshape(x(1:9),[3,3]);
pre_wd=reshape(x(10:12),[3,1]);


%反馈信息处理
Rd=reshape(u(13:21),[3,3]);

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

if norm(wd)>20
    wd=wd/norm(wd)*20;
end
% 
if norm(dwd)>100
    dwd=dwd/norm(dwd)*100;
end

sys = [reshape(Rd,[9,1]);wd;dwd];



% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,J,K)
%内部状态读取
Rd=reshape(x(1:9),[3,3]);
wd=reshape(x(10:12),[3,1]);
dwd=reshape(x(13:15),[3,1]);

%反馈信息处理
R=reshape(u(1:9),[3,3]);
w=reshape(u(10:12),[3,1]);


Rr=Rd'*R;
eR=0.5*vee(Rr-Rr');
ew=w-R'*Rd*wd;
hat_ew=hat(ew);

tmp1=Rr*hat_ew*hat(w)-hat(wd)*Rr*hat_ew- hat(dwd)*Rr;
Ad=tmp1-tmp1';

RB=[Rr(2,2)+Rr(3,3)       -Rr(2,1)          -Rr(3,1);
     -Rr(1,2)         Rr(1,1)+Rr(3,3)       -Rr(3,2);
     -Rr(1,3)           -Rr(2,3)          Rr(1,1)+Rr(2,2)];

e=vee(Rr-Rr');
de=vee(Rr*hat_ew+hat_ew*Rr');

if rank(RB)==3
    B=RB*inv(J);
    M=inv(B)*(-vee(Ad)-K*[e;de])+cross(w,J*w);
    unfullrank=0;
else
    M=-8.81/10*eR-2.54/10*ew+cross(w,J*w)-J*(hat(w)*Rr'*wd-Rr'*dwd);
    unfullrank=1;
end

M(1:3)=-8.81/10*eR-2.54/10*ew+cross(w,J*w)-J*(hat(w)*Rr'*wd-Rr'*dwd);
unfullrank=1;

% q = dcm2quat(R');
% qd = dcm2quat(Rd');
% q_inv = quatinv(q);  % 计算q的逆
% qe = quatmultiply(q_inv, qd);  % 计算q的逆与qd的乘积
% w_cmd = 8 * sign(qe(1)) * qe(2:4);
% we = w_cmd' - w;
% sys =[we;unfullrank];

th1=0.3;
th2=0.1;
M(1)=deadzone(-th1,M(1),th1);
M(2)=deadzone(-th1,M(2),th1);
M(3)=deadzone(-th2,M(3),th2);
sys =[M;unfullrank;wd];

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
