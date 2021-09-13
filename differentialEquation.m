function [xdot] = differentialEquation(t,x)
%Inputs:
%t = flight time (s)
%x = [downrange distance (m), downrange velocity (m/s), altitude (m), vertical velocity (m/s), propellant mass (kg), randomized wind speed (m/s)]
%
%Outputs:
%xdot = [downrange velocity (m/s), downrange acceleration (m/s^2) , vertical velocity (m/s), vertical acceleration (m/s^2), mass burn rate (kg/s), longitudinal dryden wind speed (m/s)]

m_payload = 2.0;                                                            %mass of payload (kg)
payload_altitude = 244;                                                     %payload deployment altitude (m)
rho = 1.225;                                                                %air density (kg/m^3)
dm = 0.075;                                                                 %motor diameter (m)
s = 0.127;                                                                  %fin half-span (m)
cr = 0.381;                                                                 %fin root chord (m)
ct = 0;                                                                     %fin tip chord (m)
ft = 0.00318;                                                               %fin thickness (m)
fn = 4;                                                                     %fin number (3 or 4)
launch_angle = (4)*pi/180;                                                  %launch angle from vertical (rad)
C_D_drogue = 1.16;                                                          %drogue parachute drag coefficient
d_drogue = 0.61;                                                            %drogue parachute diameter (m)
C_D_main = 2.20;                                                            %main parachute drag coefficient
d_main = 2.13;                                                              %main parachute diameter (m)
windspeed = 5.81152;                                                        %average windspeed (m/s)
I = 3489;                                                                   %total impuse (N*s)
mu = 18.07E-6;                                                              %dynamic viscosity (Pa*s)
L = 3.5;                                                                    %rocket length (m)
R_s = 100E-5;                                                               %surface roughness (m)

global turbulence_intensity;                                                %declare intensity of turbulence
global windupper;                                                           %declare upper wind speed
global windlower;                                                           %declare lower wind speed 
global main_altitude;                                                       %declare main parachute deployment altitude
global gamma;                                                               %declare ratio of specific heats
global R;                                                                   %declare gas constant
global T_ground;                                                            %declare temperatue on launch pad
global lapse_rate;                                                          %declare troposphere lapse rate
global g;                                                                   %declare gravitational acceleration
global rail_length;                                                         %declare rail length
global max_acceleration;                                                    %declare maximum acceleration
global m_empty;                                                             %declare unloaded mass of rocket
global d;                                                                   %declare fuselage diameter

%Calculate full mass of rocket (kg)
m = m_empty + x(5);

%Determine reference areas
A = pi*d^2/4;                                                               %rocket reference area (m^2)
A_drogue = pi*d_drogue^2/4;                                                 %drogue parachute area (m^2)
A_main = pi*d_main^2/4;                                                     %main parachute area (m^2)
A_motor = pi*dm^2/4;                                                        %motor reference area (m)

%Determine temperature
T = T_ground - lapse_rate*x(3);

%Determine Mach number
M = x(4)/sqrt(gamma*R*T);

% dryden gust model test
dgm_ode = (real(drydengustmodel_v2(x(3),x(2),x(4),turbulence_intensity)));

%Normal force coefficient
l = sqrt(cr^2+(s/2)^2);                                                     %fin half-chord (m)
alpha = (x(6)*10000+x(2))/x(4);                                             %angle of attack (rad)
C_N_nose = 2*alpha;                                                         %normal force coefficient due to nose cone
C_N_fins = alpha*4*fn*(s/d)^2/(1+sqrt(1+(2*l/(cr+ct))^2));                  %normal force coefficient due to fins
C_N = C_N_nose + C_N_fins;                                                  %total normal force coefficicent

%Time (s) and thrust (N) data
global time_data;
global thrust_data;

%Interpolate motor thrust
if t <= time_data(end)                                                      %motor is still burning
    thrust = interp1(time_data,thrust_data,t);                              %interpolate thrust (N)
else                                                                        %after burnout
    thrust = 0;                                                             %no thrust
end

%Drag coefficient
Re = rho*x(4)*L/mu;                                                         %Reynolds number
Re_cr = 51*(R_s/L)^-1.039;                                                  %surface roughness dependent critical Reynolds number                        
if Re > Re_cr                                                               %surface roughness effect
    C_f = (1-0.1*M^2)*(0.032*(R_s/L)^0.2);                                  %friction coefficient
elseif Re > 10^4                                                            %no surface roughness effect
    C_f = (1-0.1*M^2)/(1.50*log(Re)-5.6)^2;                                 %friction coefficient
else                                                                        %Reynolds number negligibly low
    C_f = 1.48E-2;                                                          %friction coefficient
end
f_B = L/d;                                                                  %fineness ratio
cm = (cr+ct)/2;                                                             %fin mean chord (m)
A_wet = pi*d*L;                                                             %body wetted area
S_wet = 4*s*cr;                                                             %fins wetted area
C_D_f = C_f*((1+1/2/f_B)*A_wet + (1+2*ft/cm)*S_wet)/A;                      %friction drag coefficient
C_D_b = 0.12 + 0.13*M^2;                                                    %base drag coefficient
Lambda = atan(cr/s);                                                        %fin leading edge angle (rad)
C_D_FLE = (cos(Lambda))^2*((1-M^2)^-.417-1);                                %fin leading edge drag coefficient
C_D_F = (4*ft*s/A)*(C_D_FLE+C_D_b);                                         %fin drag coefficient
if thrust ~= 0                                                              %motor still burning
    C_D_b = ((A - A_motor)/A)*C_D_b;                                        %motor exhaust correction
end
C_D = C_D_f + C_D_b + C_D_F;                                                %drag coefficient

%Deploy payload
if thrust == 0 && x(3) <= payload_altitude && x(4) < 0                      %payload deployment altitude has been reached during descent
    m = m - m_payload;                                                      %payload mass ejected
end

%State variable derivatives
xdot = [x(2)+x(6)*10000;(thrust*sin(launch_angle))/m;x(4);(thrust*cos(launch_angle)-m*g-.5*rho*x(4)^2*C_D*A)/m;-x(5)*thrust/I;0];

%Generate limited bandwidth noise
a = windlower;
b = windupper;
r = (b-a)*rand(1,1)+a;

%State derivative for the dryden gust model
dgm_ode(isnan(dgm_ode)) = 0;
    if (dgm_ode(2,1) ~= 0)
    xdot(6) =(((dgm_ode(1,1))*r-x(6)))/(dgm_ode(2,1)*1000);
    else
    xdot(6) = 0;
    end
    
%Accounts for delay before liftoff
if xdot(4) <= 0 && thrust ~= 0                                              %before liftoff
    xdot(2) = 0;                                                            %no downrange acceleration
    xdot(4) = 0;                                                            %no vertical acceleration
end
%Descent under drogue parachute
if x(4) <= 0 && thrust == 0                                                 %during descent
    xdot(4) = (-m*g+.5*rho*x(4)^2*C_D_drogue*A_drogue)/m;                   %vertical acceleration under drogue parachute
    %Descent under main parachute
    if x(3) < main_altitude                                                 %main deployment altitude has been reached
        xdot(4) = (-m*g+.5*rho*x(4)^2*C_D_main*A_main)/m;                   %vertical acceleration under main parachute
    end
end
%Accounts for side force due to wind
if x(3) > rail_length && x(4) > 0                                           %after launch rail exit during ascent
    xdot(2) = xdot(2)-.5*rho*x(4)^2*C_N*A;                                  %acceleration due to side force
end
%Track maximum vertical acceleration
if xdot(4) > max_acceleration && x(4) > 0                                   %new max vertical acceleration
    max_acceleration = xdot(4);                                             %update max vertical acceleration
end
end