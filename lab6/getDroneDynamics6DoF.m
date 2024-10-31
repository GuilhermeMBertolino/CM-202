function dynamics = getDroneDynamics6DoF()
% dynamics = getDroneDynamics6DoF() obtains parameters describing the
% dynamics of a 6DoF drone. The parameters are (all units in SI):
% dynamics.m: mass.
% dynamics.J: inertia matrix.
% dynamics.l: distance between the center of mass and each propeller.
% dynamics.d: distance between the center of mass and each propeller along
% the x or y body axis.
% dynamics.g: acceleration of gravity.
% dynamics.kf: thrust force constant.
% dynamics.ktau: reaction torque constant.
% dynamics.k: the ratio ktau / kf.
% dynamics.wMin: minimum velocity of each propeller.
% dynamics.wMax: maximum velocity of each propeller.
% dynamics.Gamma: matrix to transform from propeller forces
% [f1, f2, f3, f4]^T to [f, taux, tauy, tauz]^T.
% dynamics.c: radius of the propeller
% dynamics.rho: quadcopter correction parameter for in-ground-effect model

dynamics.m = 0.0630;
dynamics.J = diag([5.8286e-5, 7.1691e-5, 1e-4]);
dynamics.l = 0.0624;
dynamics.d = dynamics.l * sqrt(2.0) / 2.0;
dynamics.g = 9.81;

dynamics.c = 0.1905;
dynamics.rho = 8.6;

Ct = 0.0107;
Cq = 7.8264e-4;
r = 0.0330;
A = 0.00342;
rho = 1.184;
dynamics.kf = Ct * r^2 * A * rho;
dynamics.ktau = Cq * r^3 * A * rho;

fRotorMin = 0.0;
fRotorMax = 2.0 * dynamics.m * dynamics.g;
dynamics.wMin = sqrt(fRotorMin / dynamics.kf);
dynamics.wMax = sqrt(fRotorMax / dynamics.kf);

dynamics.k = dynamics.ktau / dynamics.kf;
d = dynamics.d;
k = dynamics.k;
dynamics.Gamma = [1.0, 1.0, 1.0, 1.0;...
                    d, -d, -d, d;...
                    -d, -d, d, d;...
                    k, -k, k, -k];

end