clear, clc
close all

force_required = 150; %N
motor_torque = 1.76*2; %Nm

radius_gear = 0.010; %needs to be more then half the distance of our tavel
radius_rod = 0.030;
off_set = 0.050;
%                                                        ||
%           0    10   20   30   40   50   60   70   80   90   100  110  120
b_length = [61.3 63.2 64.8 66.1 67.4 68.6 69.4 70.5 70.6 70.6 70.5 69.4 68.6 67.4 66.1 64.8 63.2 61.2 60  58  56.4 54.7 53.6 52.3 51.4 51  50.7 51  52.3 53.6 54.7 56.4 58  60  60.8 61.3];
%

b_angle  = [9.5  9    8.4  7.6  6.6  5.3  4.0  2.7  1.5  0    2.7  4.0  5.3  6.6  7.6  8.4  9    9.5  9.6 9.4 8.9  8.1  7.2  5.6  4    2.2 0    2.2 4    5.6  7.2  8.1  8.7 9 9.2    9.5];
a = 1;

for i = 1:1:36 ;
    angle_c(i) =  asind(sind(i*10+90)*0.05/(b_length(i)/1000));
    
end

for i = 1:1:36
    force_b(i) = motor_torque*cosd((angle_c(i)))/0.01;
end

for i = 1:1:36
    torque_b(i) = force_b(i)*b_length(i)/1000;
end

for i = 1:1:36
    force_rod_per(i) = torque_b(i)/0.03;
end

for i = 1:1:36
    force_rod(i) = force_rod_per(i)*cosd(b_angle(i));
end

for i = 1:1:36
    force_needed(i) = 150*3;
end

plot(force_rod,'r')
grid on, hold on
plot(force_needed)
axis([0 37 0 900])
title('Force applied to magnets')
xlabel('Motor angular displacement')
ylabel('Force applied to magents')
legend('Force applied','Force needed')

motor_force = motor_torque / radius_gear

applied_force = motor_force * (off_set / radius_rod)
