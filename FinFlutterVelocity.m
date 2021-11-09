%% Max Fin Flutter Velocity Calculation [imperial]
%reference: https://apogeerockets.com/education/downloads/Newsletter291.pdf
%note: simulation is in metric, but calculation is in imperial.
%note: simulation Temp and Pressure = Sea level Standard Values

function Vf = FinFlutterVelocity(Rocket,Atmos)
%Placeholder variables for L3
G = 900000;         %[lbs/in^2] (PLACEHOLDER VALUE) Shear Modulus of G10 Fiberglass Sheet
b = 8;              %[in] Fin Half Span (Height)
cr = 12;            %[in] Fin Root Chord
ct = 6;             %[in] Fin Tip Chord
t = 3/16;           %[in] Thickness
S = .5*(cr+ct)*b;   %[in^2] Area of a trapezoid
AR = (b^2)/S;       %[] Aspect Ratio
lam = ct/cr;        %[] Lambda - Chord Tip to Root Ratio
P = 14.696;         %[lbs/in^2] Air Pressure at sea level - note: pressure decreases with altitude, so this is safest value.
R = 1716.59;        %[ft-lb/(R*sl)] or [ft^2/(R*s^2)] 
T = 75;             %[F] Temperature at launch pad
a = sqrt(1.4*R*(T+459.67)); %[fps] speed of sound

% Function values
G = 900000;         %[lbs/in^2] (PLACEHOLDER VALUE) Shear Modulus of G10 Fiberglass Sheet
b = Rocket.s;       %[in] Fin Half Span (Height)
cr = Rocket.cr;            %[in] Fin Root Chord
ct = Rocket.ct;             %[in] Fin Tip Chord
t = Rocket.ft;           %[in] Thickness
S = .5*(cr+ct)*b;   %[in^2] Area of a trapezoid
AR = (b^2)/S;       %[] Aspect Ratio
lam = ct/cr;        %[] Lambda - Chord Tip to Root Ratio
P = 14.696;         %[lbs/in^2] Air Pressure at sea level - note: pressure decreases with altitude, so this is safest value.
R = 1716.59;        %[ft-lb/(R*sl)] or [ft^2/(R*s^2)] 
T = 75;             %[F] Temperature at launch pad
a = sqrt(1.4*R*(T+459.67)); %[fps] speed of sound

%% Calculation
Vf = a*sqrt(G/(((1.337*(AR^3)*P*(lam+1)))/(2*(AR+2)*(t/cr)^3)));
fprintf("Fin Flutter will occur at %.2f ft/s\n", Vf)
end

