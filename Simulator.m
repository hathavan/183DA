% Computational Simulation of a 2-wheeled robot

% Modelling the two distance sensors
% Left servo set to 180; Right servo set to 90 (Constant translational
% velocity)

Time = 500;
t = 1:Time;
% tval = zeros(1,Time);
Lidar1 = zeros(1, Time);
Lidar2 = zeros(1, Time);
Mx = zeros(1, Time);
My = zeros(1, Time);
thetaold = 0;
% Translational velocities
Vl = 0.00005; % [m/s]
Vr = 0;
V = (Vl + Vr)/2;
% angular velocity; L = 75mm
w = (1*Vl)/0.075; % [rad/s]
Dxold = Vl;
Dyold = 0;
for i = 1:Time
    thetanew = thetaold + (w*i); %[rad]
    if thetanew >= (2*pi)
        thetanew = thetanew - (2*pi);
    end
    Mx(1, i) = rad2deg(thetanew);
    orthogonal = thetanew + (pi/4);
    if orthogonal >= (2*pi)
        orthogonal = orthogonal - (2*pi);
    end
    My(1, i) = rad2deg(orthogonal);
    thetaold = thetanew;
    Dxnew = Dxold + V*cos(thetanew)*i; 
    Lidar1(1, i) = Dxnew;
    Dxold = Dxnew;
    Dynew = Dyold + V*sin(thetanew)*i;
    Lidar2(1, i) = Dynew;
    Dyold = Dynew;
end

figure(1)
plot(t, Lidar1)
title('Lidar 1')
xlabel('Time')
ylabel('Distance (m)')
figure(2)
plot(t, Lidar2)
title('Lidar 2')
xlabel('Time')
ylabel('Distance (m)')
figure(3)
plot(t, Mx)
title('Magnetometer x-direction')
xlabel('Time')
ylabel('Angle (degrees)')
figure(4)
plot(t, My)
title('Magnetometer y-direction')
xlabel('Time')
ylabel('Angle (degrees)')