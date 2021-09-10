clc; clear; close all;

%% Rocket Design Parameters
Rocket.mass_empty = 15.2171;              %unloaded mass of rocket (kg)
Rocket.mass_propellant = 2.066;           %mass of propellant (kg)
Rocket.mass_aft = 4.61;                   %mass of aft section (kg)
Rocket.mass_payload = 2.0;                %mass of payload (kg)
Rocket.main_altitude = 198;               %main parachute deployment altitude (m)

Rocket.payload_altitude = 244;            %payload deployment altitude (m)
Rocket.max_acceleration = 0;              %initialize max acceleration (m/s^2)

Rocket.length = 3.5;                      %rocket length (m)
Rocket.dia = 0.14;                        %fuselage diameter (m)
Rocket.s = 0.127;                         %fin half-span (m)
Rocket.cr = 0.381;                        %fin root chord (m)
Rocket.ct = 0;                            %fin tip chord (m)
Rocket.ft = 0.00318;                      %fin thickness (m)
Rocket.dia_motor = 0.075;                 %motor diameter (m)
Rocket.dia_drogue = 0.61;                 %drogue parachute diameter (m)
Rocket.dia_main = 2.13;                   %main parachute diameter (m)

Rocket.I = 3489;                          %total impuse (N*s)
Rocket.C_D_drogue = 1.16;                 %drogue parachute drag coefficient
Rocket.C_D_main = 2.20;                   %main parachute drag coefficient
Rocket.R_s = 100E-5;                      %surface roughness (m)

Rocket.rail_length = 12;                  %launch rail length (ft)
Rocket.launch_angle = (4)*pi/180;         %launch angle from vertical (rad)

%Load motor time (s) and thrust (N) data
Rocket.time_data = csvread('Simulation Thrust.csv',0,0,[0 0 97 0]);     %Time (s)
Rocket.thrust_data = csvread('Simulation Thrust.csv',0,1,[0 1 97 1]);   %Thrust (N)

%% Atmospheric Parameters
Atmos.g = 9.81;                         %gravitational acceleration (m/s^2)
Atmos.gamma = 1.4;                      %ratio of specific heats
Atmos.R = 287;                          %gas constant (J/kg*K)
Atmos.T_ground = 292;                   %temperature on launch pad (K)
Atmos.lapse_rate = 0.0065;              %troposphere lapse rate (K/m)
Atmos.turbulence_intensity = 1;   
Atmos.windspeed = 5.81152;              %average windspeed (m/s)
Atmos.mu = 18.07E-6;                    %dynamic viscosity (Pa*s)
Atmos.rho = 1.225;                      %air density (kg/m^3)

% Initialize upper and lower wind values for bandwidth
Atmos.windupper = 5;
Atmos.windlower = -5;

%Solve equations of motion
tspan = [0 100];                                                            %Time span
x0 = [0;0;0;0;Rocket.mass_propellant;1];                                    %Initial Conditions
OutputVars = [0,0,0,0,0,0];                                                 %OutputVars
[t,y] = ode45(@(t,y) differentialEquation(t,y,Rocket, Atmos), tspan, x0);   %ODE function

%Remove data after landing
for ii = 1:length(t)
    if y(ii,3) < 0 && y(ii,4) < 0.1 %BH: use 0.1 because initial xdot values have a slight error to them. 
        break;
    end
end

%Determine temperature throughout launch
T = Atmos.T_ground - Atmos.lapse_rate*y(:,3);

%Determine Mach number
M = sqrt(y(:,2).^2+y(:,4).^2)./sqrt(Atmos.gamma*Atmos.R*T); %this is only vertical vecloity right now. needs to be total

%Calculate ground hit kinetic energy (J)
KE = .5*Rocket.mass_aft*y(end,4)^2;

%Calculate Thrust-to-weight ratio
t2wr = mean(Rocket.thrust_data)/((Rocket.mass_empty + Rocket.mass_propellant)*Atmos.g);

%Convert to English units
y(:,1:4) = y(:,1:4)*3.28084;
Rocket.max_acceleration = Rocket.max_acceleration*3.28084;
Rocket.main_altitude = Rocket.main_altitude*3.28084;
KE = KE*0.737562;
Rocket.thrust_data = Rocket.thrust_data*0.224809;

%Aproximate alpha by converting to body axis velocities - (need to add wind
%speeds!!!) 
%
u = y(:,2).*cos(Rocket.launch_angle) + y(:,4).*sin(Rocket.launch_angle);
w = y(:,4).*cos(Rocket.launch_angle)+ y(:,2).*sin(Rocket.launch_angle);

alpha = atan(w./u);
for ii = 1:length(alpha)
    if isnan(alpha) == 1
        alpha(ii) = 0;
    end
end
figure()
plot(t, alpha)

%Determine recovery velocities (ft/s)
for ii = 1:length(t)
    if y(ii,4) < 0
        drogue_deployment_velocity = abs(y(ii+1,4));
        break;
    end
end
for ii = 1:length(t)
    if y(ii,4) < 0 && y(ii,3) <= Rocket.main_altitude
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
    if y(jj,3) >= Rocket.rail_length
        break;
    end
end
rail_exit_velocity = y(jj,4);
for kk = 1:length(t)
    if y(kk,3) > 0
        break;
    end
end
liftoff_thrust = interp1(Rocket.time_data,Rocket.thrust_data,t(kk));
drift_radius = abs(y(end,1));
descent_time = t(end) - t(apogee_index);

%Plot altitude
figure()
plot(t,y(:,3));
xlabel('Flight Time (s)');
ylabel('Altitude (ft)');

%Plot vertical velocity
figure();
plot(t,y(:,4));
xlabel('Flight Time (s)');
ylabel('Vertical Velocity (ft/s)');

%Plot thrust curve
figure();
plot(Rocket.time_data,Rocket.thrust_data);
xlabel('Flight Time (s)');
ylabel('Thrust (lbf)');

%Print mission performance predictions
fprintf('Apogee:\t\t\t\t\t\t\t\t\t%.0f ft\nMax Velocity:\t\t\t\t\t\t\t%.0f ft/s\nMax Acceleration:\t\t\t\t\t\t%.0f ft/s^2\nMax Mach Number:\t\t\t\t\t\t%.3f\nLiftoff Thrust:\t\t\t\t\t\t\t%.0f lbf\nThrust-to-Weight Ratio:\t\t\t\t\t%.2f\nRail Exit Velocity:\t\t\t\t\t\t%.0f ft/s\nImpact Kinetic Energy:\t\t\t\t\t%.0f ft*lb\nDrift Radius:\t\t\t\t\t\t\t%.0f ft\nDescent Time:\t\t\t\t\t\t\t%.0f s\nDrogue Parachute Deployment Velocity:\t%.0f ft/s\nMain Parachute Deployment Velocity:\t\t%.0f ft/s\nTerminal Velocity:\t\t\t\t\t\t%.0f ft/s\n', apogee, max_velocity, Rocket.max_acceleration, max_Mach, liftoff_thrust, t2wr, rail_exit_velocity, KE, drift_radius, descent_time, drogue_deployment_velocity, main_deployment_velocity, terminal_velocity);
