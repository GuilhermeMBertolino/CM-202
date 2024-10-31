function [nuBar_next,omegaBar_k,d_nu_next] = DFL_compensator(refs,unicycleState,v,Ts,nu_k,d_nu_k)

% Some attributions to facilitate coding
xBar = refs.xBar; 
vxBar = refs.vxBar;
yBar = refs.yBar; 
vyBar = refs.vyBar;
axBar = refs.axBar; 
ayBar = refs.ayBar;
x = unicycleState(1);
y = unicycleState(2);
theta = unicycleState(3);
vx = v(1);
vy = v(2);
   
Kp = 3;
Kd = 3;

% Write the feedforward + PD control law HERE
u1 = axBar + Kp * (xBar - x) + Kd * (vxBar - vx);
u2 = ayBar + Kp * (yBar - y) + Kd * (vyBar - vy);
      
% Compensador dynamic feedback linearization
  d_nu_next = u1*cos(theta) + u2*sin(theta);
  nuBar_next = nu_k + (d_nu_next + d_nu_k)*(Ts/2);
  omegaBar_k = (-u1*sin(theta) + u2*cos(theta))/nu_k;
    
end

