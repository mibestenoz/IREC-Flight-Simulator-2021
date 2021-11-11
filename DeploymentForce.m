function [lbsdrogue,lbsmain] = DeploymentForce(Rocket,Atmos)

%% Placeholder Values for L3 
rho = 1.225;                                %[kg/m^3]
Sd = (1.2192^2)*pi/4;                       %[m^2]
Sm = (2.4384^2)*pi/4 - (0.4064^2)*pi/4;     %[m^2] %Iris Ultra Parachute has a 16 in. spill hole 
CDd = .97;                                  %Cd for RocketMan Standard
CDm = 2.2;                                  %Cd for Fruity Chute IRIS Ultra
Vd = 30.48;                                 %[m/s] Drogue Deployment Speed ~ 100 fps
Vm = 15.24;                                 %[m/s] Main Deploy Speed       ~ 50 fps
m = 15;
g = 9.81;

%% Function Values
% rho = Atmos.rho;
% Sd = Rocket.dia_drogue*pi/4;                  %drogue parachute diameter (m)
% Sm = Rocket.dia_main*pi/4;                    %main parachute diameter (m)
% CDd = Rocket.C_D_drogue;
% CDm = Rocket.C_D_main;
% Vd = Rocket.drogue_deployment_velocity/3.28084;
% Vm = Rocket.main_deployment_velocity/3.28084;
% m = Rocket.mass_empty;
% g = Atmos.g;

%% Calculation

W = m*g;                      %[N] Weight of Rocket
Fdrogue = .5*Vd^2*Sd*rho*CDd + W; %[N] Deployment force of Drogue
Fmain = .5*Vm^2*Sm*rho*CDm + W;   %[N] Deployment force of Main

lbsdrogue = Fdrogue/4.448;
lbsmain = Fmain/4.448;
fprintf('Force of Deployment for Drogue:\t\t\t%g lbf\nForce of Deployment for Main:\t\t\t%g lbf\n', lbsdrogue, lbsmain)
end