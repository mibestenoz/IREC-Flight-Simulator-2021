clc; clear; close all;

%% Rocket Design Parameters 
%Rocket inertial parameters
Rocket.mass_empty = 19.1;             %unloaded mass of rocket (kg)
Rocket.mass_propellant = 3.77;        %mass of propellant (kg)
Rocket.mass_aft = 13.6;               %mass of aft section (kg)
Rocket.mass_payload = 4.89;              %mass of payload (kg)
Rocket.MoI_dry = 19.3;                %longitudinal moment of inertia at burnout (kg*m^2)
Rocket.cg_propellant = 2.95;          %propellant center of gravity from nose (m)

%Rocket dimensions, fin parameters, and rail button parameters
Rocket.len = 3.25;                    %rocket length (m)
Rocket.dia = 0.157;                   %fuselage diameter (m)
Rocket.s = 0.165;                     %fin height (m)
Rocket.cr = 0.381;                    %fin root chord (m)
Rocket.ct = 0.172;                    %fin tip chord (m)
Rocket.ft = 0.00406;                  %fin thickness (m)
Rocket.fn = 3;                        %number of fins
Rocket.Lambda = (45)*pi/180;          %fin sweep angle (rad)
Rocket.rbn = 2;                       %number of rail buttons
Rocket.rbA = 1.32E-4;                 %frontal area of rail button (m^2)
Rocket.rbC_D = 0.3;                   %rail button drag coefficient
Rocket.R_s = 0.00002;                 %surface roughness (m)
Rocket.dia_motor = 0.098;             %motor diameter (m)
Rocket.nose_length = 0.784;           %nose cone length (m)

%Rocket parachute parameters
Rocket.dia_drogue = 1.22;             %drogue parachute diameter (m)
Rocket.dia_main = 2.44;               %main parachute diameter (m)
Rocket.C_D_drogue = 0.97;             %drogue parachute drag coefficient
Rocket.C_D_main = 2.20;               %main parachute drag coefficient
Rocket.main_altitude = 183;           %main parachute deployment altitude (m)
Rocket.payload_altitude = 183;          %payload deployment altitude (m)

%Rocket launch rail and stability parameters
Rocket.rail_length = 5.18 ;           %launch rail length (m)
Rocket.launch_angle = (6)*pi/180;     %launch angle from vertical (rad)
Rocket.cg_loaded = 2.13;              %loaded center of gravity from nose (m)
Rocket.cp = 2.39;                     %center of pressure from nose (m)

%Motor parameters
all_data = readmatrix('Simulation Thrust.csv'); %load motor performance data
Rocket.time_data = all_data(:,1);               %time during burn (s)
Rocket.thrust_data = all_data(:,2);             %motor thrust (N)
Rocket.I = 8093;                                %total impuse (N*s)

%Tangent ogive nose cone transonic wave drag data at fineness ratio of 3
all_data = readmatrix('Transonic Wave Drag.csv'); %load wave drag data
Rocket.Mach_data = all_data(:,1);                 %Mach number
Rocket.C_D_data = all_data(:,2);                  %wave drag coefficient

%Calculate launch static stability margin (cal)
ss = (Rocket.cp-Rocket.cg_loaded)/Rocket.dia;

%% Atmospheric Parameters
Atmos.g = 9.81;                         %gravitational acceleration (m/s^2)
Atmos.gam = 1.4;                        %ratio of specific heats
Atmos.R = 287;                          %gas constant (J/kg*K)
Atmos.T_ground = 303;                   %air temperature on launch pad (K)
Atmos.P_ground = 101.0E3;               %air pressure on launch pad (Pa)
Atmos.lapse_rate = 0.0065;              %troposphere lapse rate (K/m)
Atmos.turbulence_intensity = 1;         %turbulence intensity (1 - light, 2 - moderate, 3 - severe)
Atmos.dv = 18.6E-6;                     %dynamic viscosity (Pa*s)

%Initialize upper and lower wind values for bandwidth
Atmos.windspeed = -4.47;                %average windspeed (m/s) ( (-) for rocket launching into the wind )
Atmos.windupper = -3.8;
Atmos.windlower = -3.8;

%Initialize drag coefficient vector
global CDvector;
CDvector = [];

mode = input('Run:\n(1) Detailed Simulation\n(2) Sensitivity Analysis\n');
if mode == 1
%% ODE Solver 
%Solve equations of motion
tspan = [0:0.1:300];                                                            %simulation time span (s)
x0 = [0;0;0;0;Rocket.mass_propellant;Atmos.windspeed;Rocket.launch_angle;0];     %initial conditions
[t,y] = ode45(@(t,y) differentialEquation(t,y,Rocket,Atmos),tspan,x0);           %solve ODE

%% Mission Prediction Analysis
%Load variables from struct
names = fieldnames(Rocket);
for i=1:length(names)
    eval([names{i} '=Rocket.' names{i} ';']);
end
names = fieldnames(Atmos);
for ii=1:length(names)
    eval([names{ii} '=Atmos.' names{ii} ';']);
end

%Remove data after landing
for ii = 1:length(t)                
    if y(ii,3) < 0 && y(ii,4) < 0
        t = t(1:ii-1);              %flight time (s)
        y = y(1:ii-1,:);            %flight results
        break;
    end
end

%Determine mean drag coefficient
C_D_average = mean(CDvector);

%Air temperature (K) during flight
T = T_ground - lapse_rate*y(:,3);

%Determine Mach number
V = sqrt((y(:,2)-y(:,6)).^2+y(:,4).^2);        %freestream velocity (m/s)
M = V./sqrt(gam*R*T);                          %Mach number 

%Ground hit kinetic energy (J)
KE = .5*mass_aft*V(end)^2;

%Thrust-to-weight ratio
t2wr = mean(thrust_data)/((mass_empty + mass_propellant)*g);

%Convert to English units
y(:,[1:4,6]) = y(:,[1:4,6])*3.28084;                    %(m) to (ft)
main_altitude = main_altitude*3.28084;                  %(m) to (ft)
KE = KE*0.737562;                                       %(J) to (ft*lbf)
thrust_data = thrust_data*0.224809;                     %(N) to (lbf)

%Angle of attack (deg)
alpha = 180/pi*(atan((y (:,2)-y(:,6))./y(:,4))-y(:,7));

%Determine descent results
for mm = 1:length(t)
    if y(mm,4) < 0
        drogue_deployment_velocity = abs(y(mm,4));  %velocity at drogue parachute deployment (ft/s)
        break;
    end
end
for ii = 1:length(t)
    if y(ii,4) < 0 && y(ii,3) < main_altitude
        main_deployment_velocity = abs(y(ii,4));    %velocity at main parachute deployment (ft/s)
        break;
    end
end
terminal_velocity = abs(y(end,4));                  %terminal velocity (ft/s)
drift_radius = abs(y(end,1));                       %rocket drift radius (ft)
descent_time = t(end) - t(mm);                      %descent duration (s)

%Determine ascent results
[apogee, apogee_index] = max(y(:,3));               %apogee altitude (ft)
max_velocity = max(abs(y(:,4)));                    %max velocity (ft/s)
max_Mach = max(M);                                  %max Mach number
for jj = 1:length(t)
    if y(jj,3) > rail_length*cos(launch_angle)
        rail_exit_velocity = y(jj,4);               %velocity at launch rail exit (ft/s)
        break;
    end
end
for kk = 1:length(t)
    if y(kk,3) > 0
        liftoff_thrust = interp1(time_data,thrust_data,t(kk));  %thrust at liftoff (lbf)
        break;
    end
end

%% Plot mission performance predictions
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
ylabel('Wind Velocity (ft/s)');

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

%Print results
fprintf('Apogee:\t\t\t\t\t\t\t\t\t%.0f ft\nMax Vertical Velocity:\t\t\t\t\t%.0f ft/s\nMax Mach Number:\t\t\t\t\t\t%.3f\nLiftoff Thrust:\t\t\t\t\t\t\t%.0f lbf\nThrust-to-Weight Ratio:\t\t\t\t\t%.2f\nRail Exit Velocity:\t\t\t\t\t\t%.0f ft/s\nMax Impact Kinetic Energy:\t\t\t\t%.0f ft*lbf\nDrift Radius:\t\t\t\t\t\t\t%.0f ft\nDescent Time:\t\t\t\t\t\t\t%.0f s\nDrogue Parachute Deployment Velocity:\t%.0f ft/s\nMain Parachute Deployment Velocity:\t\t%.0f ft/s\nTerminal Velocity:\t\t\t\t\t\t%.0f ft/s\nStatic Stability Margin:\t\t\t\t%.2f cal\nAverage Drag Coefficient:\t\t\t\t%.2f\n', apogee, max_velocity, max_Mach, liftoff_thrust, t2wr, rail_exit_velocity, KE, drift_radius, descent_time, drogue_deployment_velocity, main_deployment_velocity, terminal_velocity,ss,C_D_average);

%Safety checks
[lbsdrogue,lbsmain] = DeploymentForce(Rocket,Atmos); %parachute deployment forces (lbf)
Vf = FinFlutterVelocity(Rocket,Atmos);               %fin flutter velocity (ft/s)
%Verify max velocity does not exceed fin flutter velocity
if Vf < max_velocity
    fprintf('WARNING: Fins are fluttering! Fin Flutter Velocity: %g fps is lower than the Max Velocity: %g fps!', Vf,max_velocity)
end
elseif mode == 2
%% 
%Sensitivity Analysis
tspan = [0:0.1:300];                                                                %simulation time span (s)
deviation = input('Percent Deviation from Nominal Value?\n')/100;                   %percent deviation from nominal value
parameterName = input('Varying Parameter Name?\n','s');                             %name of varying parameter
samples = input('Number of Parameter Samples?\n');                                  %number of sample values tested

altitudes = [];
figure();
eval(['parameter = ' parameterName ';']);
lowerParameter = parameter*(1 - deviation);
upperParameter = parameter*(1 + deviation);
varyingValues = linspace(lowerParameter,upperParameter,samples);
legendValues = [];
for ii = 1:samples
    currentValue = varyingValues(ii);
    eval(strcat(parameterName,' = ',string(currentValue),';'));
    x0 = [0;0;0;0;Rocket.mass_propellant;Atmos.windspeed;Rocket.launch_angle;0];     %initial conditions
    [t,y] = ode45(@(t,y) differentialEquation(t,y,Rocket, Atmos), tspan, x0);
    %Plot flight profiles
    plot(t,y(:,3));  
    hold on;
    altitudes(ii)=max(y(:,3));
    legendValues = [legendValues strcat(parameterName, ':', string(varyingValues(ii)), ' (MKGS)')];
end
hold off;
xlabel('Flight Time (s)');
ylabel('Altitude (m)');
legend(legendValues);

figure();
plot(varyingValues,altitudes);
xlabel([parameterName ' (MKGS)']);
ylabel('Apogee (m)');

else
fprintf('Invalid Input\n');
end