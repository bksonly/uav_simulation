function [sys,x0,str,ts,simStateCompliance] = attitude_controller(t,x,u,flag,dt,Rd,wd,ad,J,K,dtheta)

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
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,dt,Rd,wd,ad,J,K,dtheta);

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
sizes.NumDiscStates  = 1;
sizes.NumOutputs     = 3;% M
sizes.NumInputs      = 15;%R w
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
 x0  =[0];
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
function sys=mdlUpdate(t,x,u)


sys = [x];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,dt,Rd,wd,ad,J,K,dtheta)
i=fix(t/dt)+1;
iRd=Rd(:,:,i);
iwd=wd(i,:)';
iad=ad(i,:)';

R=reshape(u(1:9),[3,3]);
w=reshape(u(10:12),[3,1]);
theta=reshape(u(13:15),[3,1]);

Rr=iRd'*R;
eR=0.5*vee(Rr-Rr');

ew=w-R'*iRd*iwd;

hat_ew=hat(ew);
tmp=Rr*hat_ew*hat(w)-hat(iwd)*Rr*hat_ew-hat(iad)*Rr;
RB=[0       -Rr(1,3)  Rr(1,2);
    Rr(2,3)  0       -Rr(2,1);
   -Rr(3,2)  Rr(3,1)  0];

e=[1;1;1]-diag(Rr);
de=-diag(Rr*hat_ew);

if rank(RB)==3
    B=RB*inv(J);
    M=inv(B)*(-diag(tmp)+K*[e;de])+cross(w,J*w);
else
    M=-8.81*eR-2.54*ew+cross(w,J*w)-J*(hat(w)*Rr'*iwd-Rr'*iad);
end
%     M=-8.81*eR-2.54*ew+cross(w,J*w)-J*(hat(w)*Rr'*iwd-Rr'*iad);

th=1;
M(1)=deadzone(-th,M(1),th);
M(2)=deadzone(-th,M(2),th);
M(3)=deadzone(-th,M(3),th);

sys =M;

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
