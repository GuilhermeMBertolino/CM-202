function [nuBar_k,omegaBar_k] = littlePoint_compensator(refs,unicycleState)

xBar = refs.xBar;
vxBar = refs.vxBar;
yBar = refs.yBar;
vyBar = refs.vyBar;
x = unicycleState(1);
y = unicycleState(2);
theta = unicycleState(3);

ell = 2;
Kp = 1;

% Write the feedforward + P control law HERE
u1 = vxBar + Kp * (xBar - x);
u2 = vyBar + Kp * (yBar - y);

% Write the compensator equations HERE
nuBar_k = cos(theta) * u1 + sin(theta) * u2;
omegaBar_k = - (1 / ell) * sin(theta) * u1 + (1 / ell) * cos(theta) * u2;

end

