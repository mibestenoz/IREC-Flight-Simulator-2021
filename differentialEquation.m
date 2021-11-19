function [xdot] = differentialEquation(t,x,Rocket,Atmos)
%% Overview
% Inputs: 
% t = flight time (s)
% x = [downrange distance (m), downrange velocity (m/s), altitude (m), vertical velocity (m/s),
%     propellant mass (kg), randomized wind velocity (m/s), pitch angle from vertical (rad), pitch rate (rad/s)]
% Rocket = rocket design parameters
% Atmos = atmospheric parameters
%
% Outputs:
% xdot = [downrange velocity (m/s), downrange acceleration (m/s^2) , vertical velocity (m/s),
%        vertical acceleration (m/s^2), mass burn rate (kg/s), longitudinal dryden wind speed (m/s), pitch rate (rad/s), pitch acceleration (rad/s^2)]

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

%Mass of loaded rocket (kg)
m = mass_empty + x(5);

%rocket center of gravity from nose
cg = (cg_loaded*(mass_empty+mass_propellant) - cg_propellant*(mass_propellant-x(5)))/(mass_empty+x(5)); %instantaneous center of gravity from nose (m)
cg_dry = (cg_loaded*(mass_empty+mass_propellant) - cg_propellant*(mass_propellant))/(mass_empty);       %center of gravity from nose at burnout (m)

%rocket longitudinal moment of inertia (kg*m^2)
MoI = MoI_dry + mass_empty*(cg-cg_dry)^2 + x(5)*(cg_propellant-cg)^2;

%Determine reference areas
A = pi*dia^2/4;                                         %rocket reference area (m^2)
A_drogue = pi*dia_drogue^2/4;                           %drogue parachute reference area (m^2)
A_main = pi*dia_main^2/4;                               %main parachute reference area (m^2)
A_motor = pi*dia_motor^2/4;                             %motor reference area (m^2)

%Air temperature (K)
T = T_ground - lapse_rate*x(3);

%Determine Mach number
V = sqrt((x(2)-x(6))^2+x(4)^2);                         %freestream velocity (m/s)
M = V/sqrt(gam*R*T);                                    %Mach number

%Motor thrust (N)
if t <= time_data(end)
    thrust = interp1(time_data,thrust_data,t);          %during burn
else
    thrust = 0;                                         %after burnout
end

%Determine normal force coefficient
l = sqrt((s*tan(Lambda)+ct/2-cr/2)^2+s^2);                      %length of fin midchord line (m)
if x(4) >= 0
    alpha = atan((x(2)-x(6))/x(4))-x(7);                        %angle of attack (rad) during ascent
else    
    alpha = atan((x(2)-x(6))/-x(4));                            %angle of attack (rad) during descent
end
C_N_nose = 2*alpha;                                             %normal force coeffciient of nose cone
C_N_fins = alpha*4*fn*(s/dia)^2/(1+sqrt(1+(2*l/(cr+ct))^2));    %normal force coefficient of fins
C_N = C_N_nose + C_N_fins;                                      %total normal force coefficient

%Dryden gust model test
u = x(2)*sin(x(7))+x(4)*cos(x(7));
w = x(2)*cos(x(7))+x(4)*sin(x(7));
dgm_ode = (real(drydengustmodel_v2(x(3),u,w,turbulence_intensity)));

%Determine drag coefficient
Re = rho*V*len/dv;                                      %Reynolds number
Re_cr = 51*(R_s/len)^-1.039;                            %surface roughness-dependent critical Reynolds number
if Re > Re_cr                                           
    C_f = (1-0.1*M^2)*(0.032*(R_s/len)^0.2);            %surface roughness-dependent skin friction coefficient
elseif Re > 10^4                                        
    C_f = (1-0.1*M^2)/(1.50*log(Re)-5.6)^2;             %fully turbulent skin friction coefficient
else                                                    
    C_f = 1.48E-2;                                      %skin friction coeffieicnt for negligibly low Reynolds number
end
f_B = len/dia;                                          %fineness ratio
cm = (cr+ct)/2;                                         %fin mean chord length (m)
A_wet = pi*dia*len;                                     %body wetted area (m^2)
S_wet = fn*s*(cr+ct);                                   %fins wetted area (m^2)
C_D_f = C_f*((1+1/2/f_B)*A_wet + (1+2*ft/cm)*S_wet)/A;  %skin friction drag coefficient
qstag = 1 + M^2/4 + M^4/40;                             %normalized stagnation pressure
C_D_stag = 0.85*qstag;                                  %stagnation pressure drag coefficient
C_D_FLE = (cos(Lambda))^2*C_D_stag;                     %fin leading edge drag coefficient
C_D_F = (fn*ft*s/A)*C_D_FLE;                            %fin drag coefficient
C_D_b = 0.12 + 0.13*M^2;                                %base drag coefficient after burnout
if thrust ~= 0
    C_D_b = ((A - A_motor)/A)*C_D_b;                    %base drag coefficient during burn
end
C_D_P = rbC_D*rbn*rbA/A;                                %parasitic drag coefficient
C_D = C_D_f + C_D_b + C_D_F + C_D_P;                    %total drag coefficient

%Deploy payload
if thrust == 0 && x(3) < payload_altitude && x(4) < 0
    m = m - mass_payload;                               %rocket mass without payload (kg)
end

%Determine drag and normal forces
Drag = .5*rho*V^2*C_D*A;                                %drag force (N)       
Norm = .5*rho*V^2*C_N*A;                                %normal force (N)

%% ODEs for Rocket 
%Rocket state derivatives during ascent
xdot(1) = x(2);                                                                     %downrange velocity (m/s)
if x(3) > cos(launch_angle)*rail_length && x(4) > 0
    xdot(2) = (thrust*sin(x(7))-Norm*cos(x(7))-Drag*sin(x(7)+alpha))/m;             %downrange acceleration off rail(m/s^2)
    xdot(4) = (thrust*cos(x(7))-m*g-Drag*cos(x(7)+alpha)+Norm*sin(x(7)))/m;         %vertical acceleration off rail (m/s^2)
    xdot(8) = (cp-cg)*(Norm+Drag*sin(alpha))/MoI;                                   %pitch acceleration off rail (rad/s^2)
else
    xdot(2) = (thrust*sin(launch_angle)-Drag*cos(alpha)*sin(launch_angle))/m;       %downrange acceleration on rail(m/s^2)   
    xdot(4) = (thrust*cos(launch_angle)-m*g-Drag*cos(alpha)*cos(launch_angle))/m;   %vertical acceleration on rail (m/s^2)
    xdot(8) = 0;                                                                    %pitch acceleration on rail (rad/s^2)
end
xdot(3) = x(4);                                                                     %vertical velocity (m/s)
xdot(5) = -mass_propellant*thrust/I;                                                           %mass burn rate (kg/s)
xdot(7) = x(8);                                                                     %pitch rate (rad/s)                                                                    
xdot = xdot';                                                                       %state derivative column vector
%% Dryden Gust Model
%Generate limited bandwidth noise
a = windlower;
b = windupper;
r = (b-a)*rand(1,1)+a;

%State derivative for the dryden gust model
dgm_ode(isnan(dgm_ode)) = 0;
    if (dgm_ode(2,1) ~= 0)
    xdot(6) =(((dgm_ode(1,1))*r-x(6)))/(dgm_ode(2,1)*10);                           %longitudinal dryden wind speed (m/s)
    else
    xdot(6) = 0;
    end
    
%% Corrections 
%Delay before liftoff
if xdot(4) <= 0 && thrust ~= 0                                      
    xdot(2) = 0;                                                    %downrange acceleration (m/s^2)
    xdot(4) = 0;                                                    %vertical acceleration (m/s^2)
end

%Descent under drogue parachute
if x(4) < 0 && thrust == 0                                          
    xdot(4) = (-m*g+.5*rho*V^2*C_D_drogue*A_drogue*cos(alpha))/m;   %vertical acceleration under drogue parachute (m/s^2)
    xdot(2) = -.5*rho*V^2*C_D_drogue*A_drogue*sin(alpha)/m;         %downrange acceleration under drogue parachute (m/s^2)
    %Descent under main parachute
    if x(3) < main_altitude                                         
        xdot(4) = xdot(4)+.5*rho*V^2*C_D_main*A_main*cos(alpha)/m;  %vertical acceleration under main parachute (m/s^2)
        xdot(2) = xdot(2)-.5*rho*V^2*C_D_main*A_main*sin(alpha)/m;  %downrange acceleration under main parachute (m/s^2)
    end
end

%End flight after ground impact 
if x(3) <= 0 && thrust == 0
    xdot(:) = 0;                                                    %state derivatives after landing
end