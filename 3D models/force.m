clear, clc
close all

force_required = 150; %N
motor_torque = 1.76; %Nm

radius_gear = 0.010; %needs to be more then half the distance of our tavel
radius_rod = 0.030;
off_set = 0.050;



motor_force = motor_torque / radius_gear

applied_force = motor_force * (off_set / radius_rod)
