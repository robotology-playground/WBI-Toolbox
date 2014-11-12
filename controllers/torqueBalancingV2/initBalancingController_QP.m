%WBC ASW
clear all

% Controller period
Ts             = 0.01; 
simulationTime = inf;
 
% Controller gains for convergence of the desired centroidal momentum. 
% The first three elements are the Proportional, Intagral, and the Derivative
% gains taking place in xComDDStart, i.e. 
%
% xComDDStart = xDDcomDes - Gains(1)*(xcom - xcomDes) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - xDcomDes)  
%
% The fourth element is the gain for the 
% angular part of the centroidal momentum convergence, i.e. 
%
% hwDot = -Gains(4)*hw  

gains  = [    120    0   1   1 ];

% Impadances acting in the null space of the desired contact forces 


impTorso            = [  70    20   10
                          0     0    0]; 
impArms             = [ 8    8    8   8   8
                        0    0    0   0   0];
impLegs             = [35   70    0    100    100  10
                        0    0   0     1000   1000  0]; 
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLegs(1,:),impLegs(1,:)];
increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLegs(2,:),impLegs(2,:)];
impedencesSat       = [100   100    1000];


              
% Rotation of the gazebo FT sensor
R   = [0 0 1; 0 -1 0;1 0 0];
Rf  = [R, zeros(3,3); zeros(3,3), R];


referenceParams = [0.05 0.4];  %[0.015 0.5];


%% Parameters for QP
number_of_feet_on_ground = 2;
init_conditions_QP   = zeros(6*number_of_feet_on_ground,1);
lower_bound_opt_var  = -inf*ones(6*number_of_feet_on_ground,1);
upper_bound_opt_var  = inf*ones(6*number_of_feet_on_ground,1);
init_params_QP       = [init_conditions_QP;lower_bound_opt_var;upper_bound_opt_var];

%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)

number_of_feet_on_ground     = 2;

% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1/3;  
torsionalFrictionCoefficient = 2/150;

%% The QP solver will search a solution fo that 
% satisfies the inequality Aineq_f F(fo) < bineq_f

[Aineq_fcone,bineq_fcone]= constraint_fcone_QP(forceFrictionCoefficient,numberOfPoints,number_of_feet_on_ground);


if number_of_feet_on_ground == 2
    Aineq_torsion = [ 0         , 0, -torsionalFrictionCoefficient,               0,               0, 1, zeros(1,6);
                      0         , 0, -torsionalFrictionCoefficient,               0,               0,-1, zeros(1,6); 
                      zeros(1,6), 0,              0,                -torsionalFrictionCoefficient, 0, 0,      1;
                      zeros(1,6), 0,              0,                -torsionalFrictionCoefficient, 0, 0,     -1];   
    bineq_torsion = zeros(4,1);
else
    Aineq_torsion = [ 0, 0, -torsionalFrictionCoefficient, 0, 0, 1;
                      0, 0, -torsionalFrictionCoefficient, 0, 0,-1];
    bineq_torsion = zeros(2,1); 
end    
%

% merge the constraints together
% here still in terms of f (not f0) 
% (the constraint on f0 are variable and calculated in the controller)
Aineq_F = [ Aineq_fcone;
            Aineq_torsion];
bineq_F = [ bineq_fcone;
            bineq_torsion];

