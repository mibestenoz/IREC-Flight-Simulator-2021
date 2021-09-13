clc; clear; close all;

%% Rocket Design Parameters
Rocket.mass_empty = 15.2171;              %unloaded mass of rocket (kg)
Rocket.mass_propellant = 2.066;           %mass of propellant (kg)
Rocket.mass_aft = 4.61;                   %mass of aft section (kg)
Rocket.mass_payload = 2.0;                %mass of payload (kg)
Rocket.main_altitude = 198;               %main parachute deployment altitude (m)

Rocket.payload_altitude = 244;            %payload deployment altitude (m)
Rocket.max_acceleration = 0;              %initialize max acceleration (m/s^2)

Rocket.len = 3.5;                         %rocket length (m)
Rocket.dia = 0.14;                        %fuselage diameter (m)
Rocket.s = 0.127;                         %fin half-span (m)
Rocket.cr = 0.381;                        %fin root chord (m)
Rocket.ct = 0;                            %fin tip chord (m)
Rocket.ft = 0.00318;                      %fin thickness (m)
Rocket.dia_motor = 0.075;                 %motor diameter (m)
Rocket.dia_drogue = 0.61;                 %drogue parachute diameter (m)
Rocket.dia_main = 2.13;                   %main parachute diameter (m)
Rocket.cg = 2.15;                         %center of gravity from nose (m)
Rocket.cp = 2.59;                         %center of pressure from nose (m)

Rocket.I = 3489;                          %total impuse (N*s)
Rocket.C_D_drogue = 1.16;                 %drogue parachute drag coefficient
Rocket.C_D_main = 2.20;                   %main parachute drag coefficient
Rocket.R_s = 100E-5;                      %surface roughness (m)

Rocket.rail_length = 12;                  %launch rail length (ft)
Rocket.launch_angle = (4)*pi/180;         %launch angle from vertical (rad)

%Load motor time (s) and thrust (N) data
Rocket.time_data = csvread('Simulation Thrust.csv',0,0,[0 0 97 0]);     %Time (s)
Rocket.thrust_data = csvread('Simulation Thrust.csv',0,1,[0 1 97 1]);   %Thrust (N)

%Calculate static stability margin (cal)
ss = (Rocket.cp-Rocket.cg)/Rocket.dia;

%% Atmospheric Parameters
Atmos.g = 9.81;                         %gravitational acceleration (m/s^2)
Atmos.gamma = 1.4;                      %ratio of specific heats
Atmos.R = 287;                          %gas constant (J/kg*K)
Atmos.T_ground = 292;                   %temperature on launch pad (K)
Atmos.lapse_rate = 0.0065;              %troposphere lapse rate (K/m)
Atmos.turbulence_intensity = 1;         %turbulence intensity (1-light,2-moderate,3-severe)
Atmos.mu = 18.07E-6;                    %dynamic viscosity (Pa*s)
Atmos.rho = 1.225;                      %air density (kg/m^3)

% Initialize upper and lower wind values for bandwidth
Atmos.windspeed = 5.8115;   %Average windspeed (m/s)( (+) for into the wind, (-) for against the wind) 
Atmos.windupper = 10;
Atmos.windlower = 0;

%% ODE Solver 
%Solve equations of motion
tspan = [0 100];                                                            %Time span
x0 = [0;0;0;0;Rocket.mass_propellant;Atmos.windspeed];                      %Initial Conditions
[t,y] = ode45(@(t,y) differentialEquation(t,y,Rocket, Atmos), tspan, x0);   %ODE function

%% Mission Analysis
%Load variables from struct
names = fieldnames(Rocket);
for i=1:length(names)
eval([names{i} '=Rocket.' names{i};]);
end
names = fieldnames(Atmos);
for ii=1:length(names)
eval([names{ii} '=Atmos.' names{ii};]);
end

%Remove data after landing
for ii = 1:length(t)
    if y(ii,3) < 0 && y(ii,4) < 0.01 %Threshold for y
        break;
    end
end

%Determine temperature throughout launch
T = T_ground - lapse_rate*y(:,3);

%Determine Mach number, Drag, and Normal force
V = sqrt(y(:,2).^2+y(:,4).^2);          %Total Velocity
M = V./sqrt(gamma*R*T);                 %Mach Number 

%Calculate ground hit vertical kinetic energy (J)
KE = .5*mass_aft*V(end).^2;

%Calculate Thrust-to-weight ratio
t2wr = mean(thrust_data)/((mass_empty + mass_propellant)*g);

%Convert to English units
y(:,1:4) = y(:,1:4)*3.28084;
max_acceleration = max_acceleration*3.28084;
main_altitude = main_altitude*3.28084;
KE = KE*0.737562;
thrust_data = thrust_data*0.224809;

%Aproximate alpha by converting to body axis velocities 
%(Change launch angle to pitch x(7))
u = y(:,4).*cos(launch_angle)+ y(:,2).*sin(launch_angle);
w = y(:,2).*cos(launch_angle) + y(:,4).*sin(launch_angle);
alpha = atan(w./u);

alphayy = (y(:,2)./y(:,4))+launch_angle;


%Determine recovery velocities (ft/s)
for ii = 1:length(t)
    if y(ii,4) < 0
        drogue_deployment_velocity = abs(y(ii+1,4));
        break;
    end
end
for ii = 1:length(t)
    if y(ii,4) < 0 && y(ii,3) <= main_altitude
        main_deployment_velocity = abs(y(ii-1,4));
        break;
    end
end
terminal_velocity = abs(y(end,4));

%Find mission performance predictions
[apogee, apogee_index] = max(y(:,3));
max_velocity = max(y(:,4));
max_Mach = max(M);
for jj = 1:length(t)
    if y(jj,3) >= rail_length
        break;
    end
end
rail_exit_velocity = y(jj,4);
for kk = 1:length(t)
    if y(kk,3) > 0
        break;
    end
end
liftoff_thrust = interp1(time_data,thrust_data,t(kk));
drift_radius = abs(y(end,1));
descent_time = t(end) - t(apogee_index);

%% Plot Results
%Plot altitude
figure()
subplot(2,3,1)
plot(t,y(:,3));
xlabel('Flight Time (s)');
ylabel('Altitude (ft)');

%Plot vertical velocity
subplot(2,3,2)
plot(t,y(:,4));
xlabel('Flight Time (s)');
ylabel('Vertical Velocity (ft/s)');

%Plot thrust curve
subplot(2,3,3)
plot(time_data,thrust_data);
xlabel('Flight Time (s)');
ylabel('Thrust (lbf)');

%Plot wind speed
subplot(2,3,4)
plot(t,y(:,6).*3.28084);
xlabel('Flight Time (s)');
ylabel('Wind Speed (ft/s)');

%Plot Mach Number
subplot(2,3,5)
plot(t,M)
xlabel('Flight Time (s)')
ylabel('Mach Number')

%Plot Angle of Attack 
subplot(2,3,6)
plot(t, alpha)
xlabel('Flight Time (s)');
ylabel('Angle of Attack (rad)');

%Print mission performance predictions
fprintf('Apogee:\t\t\t\t\t\t\t\t\t%.0f ft\nMax Velocity:\t\t\t\t\t\t\t%.0f ft/s\nMax Acceleration:\t\t\t\t\t\t%.0f ft/s^2\nMax Mach Number:\t\t\t\t\t\t%.3f\nLiftoff Thrust:\t\t\t\t\t\t\t%.0f lbf\nThrust-to-Weight Ratio:\t\t\t\t\t%.2f\nRail Exit Velocity:\t\t\t\t\t\t%.0f ft/s\nGround Impact Kinetic Energy:\t\t\t\t\t%.0f ft*lb\nDrift Radius:\t\t\t\t\t\t\t%.0f ft\nDescent Time:\t\t\t\t\t\t\t%.0f s\nDrogue Parachute Deployment Velocity:\t%.0f ft/s\nMain Parachute Deployment Velocity:\t\t%.0f ft/s\nTerminal Velocity:\t\t\t\t\t\t%.0f ft/s\nStatic Stability Margin:\t\t\t\t%.2f cal\n', apogee, max_velocity, max_acceleration, max_Mach, liftoff_thrust, t2wr, rail_exit_velocity, KE, drift_radius, descent_time, drogue_deployment_velocity, main_deployment_velocity, terminal_velocity,ss);
