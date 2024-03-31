function [sys,x0,str,ts,simStateCompliance] = se3controller(t,x,u,flag)
%SFUNTMPL General MATLAB S-Function Template 
%   With MATLAB S-functions, you can define you own ordinary differential
%   equations (ODEs), discrete system equations, and/or just about
%   any type of algorithm to be used within a Simulink block diagram.
%
%   The general form of an MATLAB S-function syntax is:
%       [SYS,X0,STR,TS,SIMSTATECOMPLIANCE] = SFUNC(T,X,U,FLAG,P1,...,Pn)
%
%   What is returned by SFUNC at a given point in time, T, depends on the
%   value of the FLAG, the current state vector, X, and the current
%   input vector, U.
%
%   FLAG   RESULT             DESCRIPTION
%   -----  ------             --------------------------------------------
%   0      [SIZES,X0,STR,TS]  Initialization, return system sizes in SYS,
%                             initial state in X0, state ordering strings
%                             in STR, and sample times in TS.
%   1      DX                 Return continuous state derivatives in SYS.
%   2      DS                 Update discrete states SYS = X(n+1)
%   3      Y                  Return outputs in SYS.
%   4      TNEXT              Return next time hit for variable step sample
%                             time in SYS.
%   5                         Reserved for future (root finding).
%   9      []                 Termination, perform any cleanup SYS=[].
%
%
%   The state vectors, X and X0 consists of continuous states followed
%   by discrete states.
%
%   Optional parameters, P1,...,Pn can be provided to the S-function and
%   used during any FLAG operation.
%
%   When SFUNC is called with FLAG = 0, the following information
%   should be returned:
%
%      SYS(1) = Number of continuous states.
%      SYS(2) = Number of discrete states.
%      SYS(3) = Number of outputs.
%      SYS(4) = Number of inputs.
%               Any of the first four elements in SYS can be specified
%               as -1 indicating that they are dynamically sized. The
%               actual length for all other flags will be equal to the
%               length of the input, U.
%      SYS(5) = Reserved for root finding. Must be zero.
%      SYS(6) = Direct feedthrough flag (1=yes, 0=no). The s-function
%               has direct feedthrough if U is used during the FLAG=3
%               call. Setting this to 0 is akin to making a promise that
%               U will not be used during FLAG=3. If you break the promise
%               then unpredictable results will occur.
%      SYS(7) = Number of sample times. This is the number of rows in TS.
%
%
%      X0     = Initial state conditions or [] if no states.
%
%      STR    = State ordering strings which is generally specified as [].
%
%      TS     = An m-by-2 matrix containing the sample time
%               (period, offset) information. Where m = number of sample
%               times. The ordering of the sample times must be:
%
%               TS = [0      0,      : Continuous sample time.
%                     0      1,      : Continuous, but fixed in minor step
%                                      sample time.
%                     PERIOD OFFSET, : Discrete sample time where
%                                      PERIOD > 0 & OFFSET < PERIOD.
%                     -2     0];     : Variable step discrete sample time
%                                      where FLAG=4 is used to get time of
%                                      next hit.
%
%               There can be more than one sample time providing
%               they are ordered such that they are monotonically
%               increasing. Only the needed sample times should be
%               specified in TS. When specifying more than one
%               sample time, you must check for sample hits explicitly by
%               seeing if
%                  abs(round((T-OFFSET)/PERIOD) - (T-OFFSET)/PERIOD)
%               is within a specified tolerance, generally 1e-8. This
%               tolerance is dependent upon your model's sampling times
%               and simulation time.
%
%               You can also specify that the sample time of the S-function
%               is inherited from the driving block. For functions which
%               change during minor steps, this is done by
%               specifying SYS(7) = 1 and TS = [-1 0]. For functions which
%               are held during minor steps, this is done by specifying
%               SYS(7) = 1 and TS = [-1 1].
%
%      SIMSTATECOMPLIANCE = Specifices how to handle this block when saving and
%                           restoring the complete simulation state of the
%                           model. The allowed values are: 'DefaultSimState',
%                           'HasNoSimState' or 'DisallowSimState'. If this value
%                           is not speficified, then the block's compliance with
%                           simState feature is set to 'UknownSimState'.


%   Copyright 1990-2010 The MathWorks, Inc.

%
% The following outlines the general structure of an S-function.
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

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
    sys=mdlOutputs(t,x,u);

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
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

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
sizes.NumDiscStates  = 24;%xd vd ad Rd wd dwd
sizes.NumOutputs     = 4;%f M
sizes.NumInputs      = 24;%xd b1d x v R-1 wb
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
 x0  =[0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 0 0 0]';
%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
global dt
dt=0.001;
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
global m g dt
kx=16;
kv=5.6;

xd=u(1:3);
vd=(xd-x(1:3))/dt;
ad=(vd-x(4:6))/dt;

b1d=u(4:6);
xe=u(7:9);
ve=u(10:12);

ex=xe-xd;
ev=ve-vd;

tmp=-kx*ex-kv*ev-m*g*[0;0;1]+m*ad;
b3d=-tmp/norm(tmp);
b2d=cross(b3d,b1d)/norm(cross(b3d,b1d));
Rd=[cross(b2d,b3d),b2d,b3d];

if(t==0)
    dRd=zeros([3,3]);
else
    dRd=(Rd-reshape(x(10:18),[3,3]))/dt;%%problem occur
end

wd=vee(Rd\dRd);

dwd=(wd-x(19:21))/dt;

sys = [xd; vd; ad; reshape(Rd,[9,1]);wd;dwd];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
global m g J;

% ky=4;
kx=3;
kv=2;
kR=2;
kw=1;

xd=x(1:3);
vd=x(4:6);
ad=x(7:9);
Rd=reshape(x(10:18),[3,3]);
wd=x(19:21);
dwd=x(22:24);

xe=u(7:9);
ve=u(10:12);
w=u(22:24);
R=inv(reshape(u(13:21),[3,3]));
% s=u(13);
% q=u(14:16);
% Q=[0 -q(3) q(2);
%    q(3) 0 -q(1);
%    -q(2) q(1) 0];
% 
% R=q*q'+s*s*eye(3)+2*s*Q+Q*Q;
% j1=u(13);
% j2=u(14);
% j3=u(15);
% R=[cos(j3)*cos(j2),cos(j3)*sin(j2)*sin(j1)-cos(j1)*sin(j3),cos(j3)*sin(j2)*cos(j1)+sin(j1)*sin(j3);
%    sin(j3)*cos(j2),sin(j3)*sin(j2)*sin(j1)+cos(j1)*cos(j3),sin(j3)*sin(j2)*cos(j1)-sin(j1)*cos(j3)   ;
%     -sin(j2),sin(j1)*cos(j2),cos(j1)*cos(j2)];



ex=xe-xd;
ev=ve-vd;
e3=[0;0;1];
% ad=ad/norm(ad)*deadzone(-10,norm(ad),10);
f=(-ex*kx-ev*kv-e3*m*g+ad*m)' *(R*[0;0;1]);
f=min(f,m*g*2);
% disp(ex)
% if(t==2e-4)
%     fprintf("ad")
%     disp(ad)
%     fprintf("f")
%     disp(f)
%     fprintf("ev")
%     disp(ev)
% end
eR=vee((Rd'*R-R'*Rd)/2);
ew=w-R'*Rd*wd;
dwd=deadzone(-0.16,dwd,0.16);
M=-kR*eR-kw*ew+cross(w,J*w)-J*(cross(w,R'*Rd*wd)-R'*Rd*dwd);
% M=-kR*eR-kw*ew+cross(w,J*w);
M(1)=deadzone(-0.01,M(1),0.01);
M(2)=deadzone(-0.01,M(2),0.01);
M(3)=deadzone(-0.0008,M(3),0.0008);

sys =[f;M];

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
