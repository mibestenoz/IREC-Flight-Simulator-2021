function [xdot] = differentialEquation(t,x,Rocket,Atmos)
%% Overview
% Inputs: 
% t - flight time (s)
% x = [downrange distance (m), downrange velocity (m/s), altitude (m), vertical velocity (m/s),
%     propellant mass (kg), randomized wind speed (m/s)]
% Two structs with input parameters:
%   Rocket - contains all rocket vars including the payload and recovery parameters
%   Atmos - which contains the parameters of the atmosphere and wind
%
% Outputs:
% xdot = [downrange velocity (m/s), downrange acceleration (m/s^2) , vertical velocity (m/s),
%        vertical acceleration (m/s^2), mass burn rate (kg/s), longitudinal dryden wind speed (m/s)]

%% Initialize ODE parameters 
%Dynamically load variables from struct 
names = fieldnames(Rocket);
for i=1:length(names)
    eval([names{i} '=Rocket.' names{i}; ])
end

names = fieldnames(Atmos);
for ii=1:length(names)
   eval([names{ii} '=Atmos.' names{ii}; ]) 
end

%Current Mass of rocket
m = mass_empty + x(5);                                  %full mass of rocket (kg)

%Determine reference areas
A = pi*dia^2/4;                                         %rocket reference area (m^2)
A_drogue = pi*dia_drogue^2/4;                           %drogue parachute area (m^2)
A_main = pi*dia_main^2/4;                               %main parachute area (m^2)
A_motor = pi*dia_motor^2/4;                             %motor reference area (m)

%Determine temperature
T = T_ground - lapse_rate*x(3);

%Determine Mach number
V = sqrt((x(2)-x(6))^2+x(4)^2);
M = V/sqrt(gam*R*T);

%interpolate motor thrust
if t <= time_data(end)
    thrust = interp1(time_data,thrust_data,t);
else
    thrust = 0;
end

%Normal force coefficient
l = sqrt((cr/2)^2+s^2);                                 %half-span at fin half-chord (m)
if x(4) >= 0
    alpha = atan((x(2)-x(6))/x(4))-launch_angle;
else    
    alpha = atan((x(2)-x(6))/-x(4));
end
C_N_nose = 2*alpha;
C_N_fins = alpha*4*fn*(s/dia)^2/(1+sqrt(1+(2*l/(cr+ct))^2));
C_N = C_N_nose + C_N_fins;

% dryden gust model test
u = x(2)*sin(launch_angle)+x(4)*cos(launch_angle);
w = x(2)*cos(launch_angle)+x(4)*sin(launch_angle);
dgm_ode = (real(drydengustmodel_v2(x(3),u,w,turbulence_intensity)));

%Drag coefficient
Re = rho*V*len/dv;                                      %Reynolds number
Re_cr = 51*(R_s/len)^-1.039;                            %surface roughness dependent critical Reynolds number
if Re > Re_cr                                           %surface roughness effect
    C_f = (1-0.1*M^2)*(0.032*(R_s/len)^0.2);            %friction coefficient
elseif Re > 10^4                                        %no surface roughness effect
    C_f = (1-0.1*M^2)/(1.50*log(Re)-5.6)^2;
else                                                    %Reynolds number negligibly low
    C_f = 1.48E-2;
end
f_B = len/dia;                                          %fineness ratio
cm = (cr+ct)/2;                                         %fin mean chord (m)
A_wet = pi*dia*len;                                     %body wetted area
S_wet = fn*s*cr;                                        %fins wetted area
C_D_f = C_f*((1+1/2/f_B)*A_wet + (1+2*ft/cm)*S_wet)/A;  %friction drag coefficient
C_D_b = 0.12 + 0.13*M^2;                                %base drag coefficient
Lambda = atan(cr/s);                                    %fin leading edge angle (rad)
qstag = 1 + M^2/4 + M^4/40;                             %normalized stagnation pressure
C_D_stag = 0.85*qstag;                                  %stagnation pressure drag coefficient
C_D_FLE = (cos(Lambda))^2*C_D_stag;                     %fin leading edge drag coefficient
C_D_F = (fn*ft*s/A)*(C_D_FLE+C_D_b);                    %fin drag coefficient
if thrust ~= 0
    C_D_b = ((A - A_motor)/A)*C_D_b;                    %motor exhaust correction
end
C_D_P = rbC_D*rbn*rbA/A;                                %parasitic drag coefficient
C_D = C_D_f + C_D_b + C_D_F + C_D_P;                    %drag coefficient

%Deploy payload
if thrust == 0 && x(3) < payload_altitude && x(4) < 0
    m = m - mass_payload;
end

%Define Drag and Normal Forces
Drag = .5*rho*V^2*C_D*A;
Norm = .5*rho*V^2*C_N*A;

%% ODEs for Rocket 
%State-space representation
xdot(1) = x(2);                                                                                    %downrange velocity (m/s)
if x(3) > cos(launch_angle)*rail_length && x(4) > 0
    xdot(2) = (thrust*sin(launch_angle)-Norm*cos(launch_angle)-Drag*sin(launch_angle+alpha))/m;    %downrange acceleration off rail(m/s^2)
    xdot(4) = (thrust*cos(launch_angle)-m*g-Drag*cos(launch_angle+alpha)+Norm*sin(launch_angle))/m;%vertical acceleration off rail (m/s^2)
else
    xdot(2) = (thrust*sin(launch_angle)-Drag*cos(alpha)*sin(launch_angle))/m;                      %downrange acceleration on rail(m/s^2)   
    xdot(4) = (thrust*cos(launch_angle)-m*g-Drag*cos(alpha)*cos(launch_angle))/m;                  %vertical acceleration on rail (m/s^2)
end
xdot(3) = x(4);                                                                                    %vertical velocity (m/s)
xdot(5) = -x(5)*thrust/I;                                                                          %mass burn rate (kg/s)
xdot(6) = 0;                                                                                       %longitudinal dryden wind speed (m/s)
xdot = xdot';
%% Dryden Gust Model
%Generate limited bandwidth noise
a = windlower;
b = windupper;
r = (b-a)*rand(1,1)+a;

%State derivative for the dryden gust model
dgm_ode(isnan(dgm_ode)) = 0;
    if (dgm_ode(2,1) ~= 0)
    xdot(6) =(((dgm_ode(1,1))*r-x(6)))/(dgm_ode(2,1)*10);
    else
    xdot(6) = 0;
    end
    
%% Corrections 
%Accounts for delay before liftoff
if xdot(4) <= 0 && thrust ~= 0                                      %before liftoff
    xdot(2) = 0;                                                    %no downrange acceleration
    xdot(4) = 0;                                                    %no vertical acceleration
end

%Descent under drogue parachute
if x(4) < 0 && thrust == 0                                          %during descent
    xdot(4) = (-m*g+.5*rho*V^2*C_D_drogue*A_drogue*cos(alpha))/m;   %vertical acceleration under drogue parachute
    xdot(2) = -.5*rho*V^2*C_D_drogue*A_drogue*sin(alpha)/m;
    %Descent under main parachute
    if x(3) < main_altitude                                         %main deployment altitude has been reached
        xdot(4) = xdot(4)+.5*rho*V^2*C_D_main*A_main*cos(alpha)/m;  %vertical acceleration under main parachute
        xdot(2) = xdot(2)-.5*rho*V^2*C_D_main*A_main*sin(alpha)/m;
    end
end

%End flight after ground impact 
if x(3) <= 0 && thrust == 0
    xdot(:) = 0;
end