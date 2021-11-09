clc; clear; close all;

%% Rocket Design Parameters
Rocket.mass_empty = 15.36;                %unloaded mass of rocket (kg)
Rocket.mass_propellant = 2.066;           %mass of propellant (kg)
Rocket.mass_aft = 6.23;                   %mass of aft section (kg)
Rocket.mass_payload = 2.01;               %mass of payload (kg)
Rocket.main_altitude = 198;               %main parachute deployment altitude (m)
Rocket.MoI = 22.0;                        %longitudinal moment of inertia (kg*m^2)

Rocket.payload_altitude = 244;            %payload deployment altitude (m)

Rocket.len = 3.5;                         %rocket length (m)
Rocket.dia = 0.14;                        %fuselage diameter (m)
Rocket.s = 0.127;                         %fin half-span (m)
Rocket.cr = 0.381;                        %fin root chord (m)
Rocket.ct = 0;                            %fin tip chord (m)
Rocket.ft = 0.00318;                      %fin thickness (m)
Rocket.fn = 4;                            %number of fins
Rocket.rbn = 2;                           %number of rail buttons
Rocket.rbA = 1.32E-4;                     %frontal area of rail button (m^2)
Rocket.rbC_D = 0.3;                       %rail button drag coefficient
Rocket.dia_motor = 0.075;                 %motor diameter (m)
Rocket.dia_drogue = 0.61;                 %drogue parachute diameter (m)
Rocket.dia_main = 2.13;                   %main parachute diameter (m)
Rocket.cg = 2.15;                         %center of gravity from nose (m)
Rocket.cp = 2.59;                         %center of pressure from nose (m)

Rocket.I = 3489;                          %total impuse (N*s)
Rocket.C_D_drogue = 1.16;                 %drogue parachute drag coefficient
Rocket.C_D_main = 2.20;                   %main parachute drag coefficient
Rocket.R_s = 71E-5;                       %surface roughness (m)

Rocket.rail_length = 2.44;                %launch rail length (m)
Rocket.launch_angle = (5)*pi/180;         %launch angle from vertical (rad)

%Load motor time (s) and thrust (N) data
Rocket.time_data = csvread('Simulation Thrust.csv',0,0,[0 0 97 0]);     %Time (s)
Rocket.thrust_data = csvread('Simulation Thrust.csv',0,1,[0 1 97 1]);   %Thrust (N)

%Calculate static stability margin (cal)
ss = (Rocket.cp-Rocket.cg)/Rocket.dia;

%% Atmospheric Parameters
Atmos.g = 9.81;                         %gravitational acceleration (m/s^2)
Atmos.gam = 1.4;                        %ratio of specific heats
Atmos.R = 287;                          %gas constant (J/kg*K)
Atmos.T_ground = 292;                   %temperature on launch pad (K)
Atmos.lapse_rate = 0.0065;              %troposphere lapse rate (K/m)
Atmos.turbulence_intensity = 1;         %turbulence intensity (1-light,2-moderate,3-severe)
Atmos.dv = 18.07E-6;                    %dynamic viscosity (Pa*s)
Atmos.rho = 1.225;                      %air density (kg/m^3)

% Initialize upper and lower wind values for bandwidth
Atmos.windspeed = -5.8115;              %Average windspeed (m/s)( (-) for against the wind) )
Atmos.windupper = -5.8115;
Atmos.windlower = -5.8115;

%% ODE Solver 
%Solve equations of motion
tspan = [0 100];                                                            %Time span
x0 = [0;0;0;0;Rocket.mass_propellant;Atmos.windspeed;Rocket.launch_angle;0];%Initial Conditions
[t,y] = ode45(@(t,y) differentialEquation(t,y,Rocket,Atmos),tspan,x0);      %ODE function

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
    if y(ii,3) < 0 && y(ii,4) < 0
        t = t(1:ii-1);
        y = y(1:ii-1,:);
        break;
    end
end

%Determine temperature throughout launch
T = T_ground - lapse_rate*y(:,3);

%Determine Mach number
V = sqrt((y(:,2)-y(:,6)).^2+y(:,4).^2);        %Freestream Velocity
M = V./sqrt(gam*R*T);                          %Mach Number 

%Calculate ground hit kinetic energy (J)
KE = .5*mass_aft*V(end).^2;

%Calculate Thrust-to-weight ratio
t2wr = mean(thrust_data)/((mass_empty + mass_propellant)*g);

%Convert to English units
y(:,1:4) = y(:,1:4)*3.28084;
y(:,6) = y(:,6)*3.28084;
main_altitude = main_altitude*3.28084;
KE = KE*0.737562;
thrust_data = thrust_data*0.224809;

%Calculate angle of attack (deg)
alpha = 180/pi*(atan((y (:,2)-y(:,6))./y(:,4))-y(:,7));

%Determine recovery velocities (ft/s)
for mm = 1:length(t)
    if y(mm,4) < 0
        drogue_deployment_velocity = abs(y(mm,4));
        break;
    end
end
for ii = 1:length(t)
    if y(ii,4) < 0 && y(ii,3) < main_altitude
        main_deployment_velocity = abs(y(ii,4));
        break;
    end
end
terminal_velocity = abs(y(end,4));

Rocket.main_deployment_velocity = main_deployment_velocity;
Rocket.drogue_deployment_velocity = drogue_deployment_velocity;
[lbsdrogue,lbsmain] = DeploymentForce(Rocket,Atmos);

Vf = FinFlutterVelocity(Rocket,Atmos);

%Find mission performance predictions
[apogee, apogee_index] = max(y(:,3));
max_velocity = max(abs(y(:,4)));
max_Mach = max(M);
for jj = 1:length(t)
    if y(jj,3) > rail_length*cos(launch_angle)
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
descent_time = t(end) - t(mm);

%Verify max velocity does not exceed fin flutter velocity
if Vf < max_velocity
    fprintf('WARNING: Fins are fluttering! Fin Flutter Velocity: %g fps is lower than the Max Velocity: %g fps!', Vf,max_velocity)
end

%% Plot Results
%Plot altitude
figure()
subplot(3,3,1)
plot(t,y(:,3));
xlabel('Flight Time (s)');
ylabel('Altitude (ft)');

%Plot vertical velocity
subplot(3,3,2)
plot(t,y(:,4));
xlabel('Flight Time (s)');
ylabel('Vertical Velocity (ft/s)');

%Plot thrust curve
subplot(3,3,3)
plot(time_data,thrust_data);
xlabel('Flight Time (s)');
ylabel('Thrust (lbf)');

%Plot wind velocity
subplot(3,3,4)
plot(t,y(:,6));
xlabel('Flight Time (s)');
ylabel('Wind Speed (ft/s)');

%Plot Mach number
subplot(3,3,5)
plot(t,M)
xlabel('Flight Time (s)')
ylabel('Mach Number')

%Plot angle of attack 
subplot(3,3,6)
plot(t(1:mm-1),alpha(1:mm-1))
xlabel('Flight Time (s)');
ylabel('Angle of Attack (deg)');

%Plot pitch angle
subplot(3,3,7)
plot(t(1:apogee_index),180/pi*y(1:apogee_index,7))
xlabel('Flight Time (s)');
ylabel('Pitch Angle (deg)');

%Print mission performance predictions
fprintf('\nApogee:\t\t\t\t\t\t\t\t\t%.0f ft\nMax Vertical Velocity:\t\t\t\t\t%.0f ft/s\nMax Mach Number:\t\t\t\t\t\t%.3f\nLiftoff Thrust:\t\t\t\t\t\t\t%.0f lbf\nThrust-to-Weight Ratio:\t\t\t\t\t%.2f\nRail Exit Velocity:\t\t\t\t\t\t%.0f ft/s\nMax Impact Kinetic Energy:\t\t\t\t%.0f ft*lbf\nDrift Radius:\t\t\t\t\t\t\t%.0f ft\nDescent Time:\t\t\t\t\t\t\t%.0f s\nDrogue Parachute Deployment Velocity:\t%.0f ft/s\nMain Parachute Deployment Velocity:\t\t%.0f ft/s\nTerminal Velocity:\t\t\t\t\t\t%.0f ft/s\nStatic Stability Margin:\t\t\t\t%.2f cal\n', apogee, max_velocity, max_Mach, liftoff_thrust, t2wr, rail_exit_velocity, KE, drift_radius, descent_time, drogue_deployment_velocity, main_deployment_velocity, terminal_velocity,ss);

%simulate and average 100 flights
% altitudes = []
% figure()
% hold on
% tic
% for ii = 1:10
%     [t,y] = ode45(@(t,y) differentialEquation(t,y,Rocket, Atmos), tspan, x0)
%     %Plot altitude
%     plot(t,y(:,3).*3.28084);   
%     altitudes(ii)=max(y(:,3))*3.28084;
% end
% toc
% xlabel('Flight Time (s)');
% ylabel('Altitude (ft)')
% for ii = 1:10
%     fprintf('Altitude #%g: %g ft \n',ii,altitudes(ii))
% end
% fprintf('Mean Altitudes: %g ft',mean(altitudes))
    