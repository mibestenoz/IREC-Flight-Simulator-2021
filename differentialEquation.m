function [xdot] = differentialEquation(t,x,Rocket, Atmos)
%% Overview
% Inputs: 
% InputVars stores two structs with the parameters:
%   Rocket - contains all rocket vars including the payload and recovery parameters
%   Atmos - which containsthe parameters of the atmosphere
% t - flight time (s)
% x = [downrange distance (m), downrange velocity (m/s), altitude (m), vertical velocity (m/s),
%     propellant mass (kg), randomized wind speed (m/s)]
%
% Outputs:
% xdot = [downrange velocity (m/s), downrange acceleration (m/s^2) , vertical velocity (m/s),
%        vertical acceleration (m/s^2), mass burn rate (kg/s), longitudinal dryden wind speed (m/s)]

%% Initialize ODE parameters 
%Current Mass of rocket
m = Rocket.mass_empty + x(5);                                 %full mass of rocket (kg)

%Determine reference areas
A = pi*Rocket.dia^2/4;                                       %rocket reference area (m^2)
A_drogue = pi*Rocket.dia_drogue^2/4;                         %drogue parachute area (m^2)
A_main = pi*Rocket.dia_main^2/4;                             %main parachute area (m^2)
A_motor = pi*Rocket.dia_motor^2/4;                           %motor reference area (m)

%Determine temperature
T = Atmos.T_ground - Atmos.lapse_rate*x(3);

%Determine Mach number
V = sqrt(x(2)^2+x(4)^2);
M = V/sqrt(Atmos.gamma*Atmos.R*T);

%Normal force coefficient
l = sqrt(Rocket.cr^2+(Rocket.s/2)^2);                        %fin half-chord (m)
if x(4) == 0 && x(2) == 0
    alpha = 0;                                               %angle of attack (rad) - u divided by v
else 
    alpha = (x(6)+x(2))/x(4);
end
C_N_nose = 2*alpha;
C_N_fins = alpha*16*(Rocket.s/Rocket.dia)^2/(1+sqrt(1+(2*Rocket.length/(Rocket.cr+Rocket.ct))^2));
C_N = C_N_nose + C_N_fins;

% dryden gust model test
dgm_ode = (real(drydengustmodel_v2(x(3),x(2),x(4),Atmos.turbulence_intensity)));

%interpolate motor thrust
if t <= Rocket.time_data(end)
    thrust = interp1(Rocket.time_data,Rocket.thrust_data,t);
else
    thrust = 0;
end

%Drag coefficient
Re = Atmos.rho*V*Rocket.length/Atmos.mu;                            %Reynolds number
Re_cr = 51*(Rocket.R_s/Rocket.length)^-1.039;                          %surface roughness dependent critical Reynolds number
if Re > Re_cr
    C_f = (1-0.1*M^2)*(0.032*(Rocket.R_s/Rocket.length)^0.2);          %friction coefficient
elseif Re > 10^4
    C_f = (1-0.1*M^2)/(1.50*log(Re)-5.6)^2;
else
    C_f = 1.48E-2;
end
f_B = Rocket.length/Rocket.dia;                                         %fineness ratio
cm = (Rocket.cr+Rocket.ct)/2;                                           %fin mean chord (m)
A_wet = pi*Rocket.dia*Rocket.length;                                    %body wetted area
S_wet = 4*Rocket.s*Rocket.cr;                                           %fins wetted area
C_D_f = C_f*((1+1/2/f_B)*A_wet + (1+2*Rocket.ft/cm)*S_wet)/A;           %friction drag coefficient
C_D_b = 0.12 + 0.13*M^2;                                                %base drag coefficient
Lambda = atan(Rocket.cr/Rocket.s);                                      %fin leading edge angle (rad)
C_D_FLE = (cos(Lambda))^2*((1-M^2)^-.417-1);                            %fin leading edge drag coefficient
C_D_F = (4*Rocket.ft*Rocket.s/A)*(C_D_FLE+C_D_b);                       %fin drag coefficient
if thrust ~= 0
    C_D_b = ((A - A_motor)/A)*C_D_b;                                    %motor exhaust correction
end
C_D = C_D_f + C_D_b + C_D_F;                                            %drag coefficient

%deploy payload
if thrust == 0 && x(3) <= Rocket.payload_altitude && x(4) < 0
    m = m - Rocket.mass_payload;
end

Drag = .5*Atmos.rho*V^2*C_D*A;
Normal = .5*Atmos.rho*V^2*C_N*A;

%% ODEs for Rocket 
%State-space representation
xdot(1,:) = x(2);                                                             %downrange velocity (m/s)
if x(3) > Rocket.rail_length && x(4) > 0
    xdot(2,:) = (thrust*sin(Rocket.launch_angle)-Normal*cos(Rocket.launch_angle)-Drag*sin(Rocket.launch_angle))/m;  %downrange acceleration off rail(m/s^2)
else
    xdot(2,:) = (thrust*sin(Rocket.launch_angle))/m;                          %downrange acceleration on rail(m/s^2)    
end
xdot(3,:) = x(4);                                                             %vertical velocity (m/s)
xdot(4,:) = (thrust*cos(Rocket.launch_angle)-m*Atmos.g-Drag*cos(Rocket.launch_angle)+Normal*sin(Rocket.launch_angle))/m;    %vertical acceleration (m/s^2)
xdot(5,:) = -x(5)*thrust/Rocket.I;                                            %mass burn rate (kg/s)
xdot(6,:) = 0;                                                                %longitudinal dryden wind speed (m/s)

%% Dryden Gust Model
%Generate limited bandwidth noise
a = Atmos.windlower;
b = Atmos.windupper;
r = (b-a)*rand(1,1)+a;

%State derivative for the dryden gust model
dgm_ode(isnan(dgm_ode)) = 0;
    if (dgm_ode(2,1) ~= 0)
    xdot(6) =(((dgm_ode(1,1))*r-x(6)))/(dgm_ode(2,1)*1000);
    else
    xdot(6) = 0;
    end
    
%% Corrections 
%Accounts for delay before liftoff
if xdot(4) <= 0 && thrust ~= 0
    xdot(2) = 0;
    xdot(4) = 0;
end

%Descent under drogue parachute
% if x(4) < 0 && thrust == 0
%     xdot(4) = (-m*Atmos.g+.5*Atmos.rho*x(4)^2*Rocket.C_D_drogue*A_drogue)/m;
%     %Descent under main parachute
%     if x(3) < Rocket.main_altitude
%         xdot(4) = (-m*Atmos.g+.5*Atmos.rho*x(4)^2*Rocket.C_D_main*A_main)/m;
%     end
% end

%Accounts for side force due to wind

%Track maximum acceleration
if xdot(4) > Rocket.max_acceleration && x(4) > 0
    Rocket.max_acceleration = xdot(4);
end

%End the flight if below the ground and thrust is 0. 
if x(3) < -0.001 && thrust ==0
    xdot(:) = 0;
end
end
