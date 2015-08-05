clear, clc


horn = 0.025;
link = 0.070;
pickup_l = 0.085;
pickup_h = 0.040;
servo_l = 0.030;
servo_h = 0.110;
cog = 0.070;

torque = 1*2;  %Nm
mass = 7;


f_l = torque/horn;
link_applied_force = f_l*sind(80);
link_pivit_lenght = sqrt((pickup_l)^2+(pickup_h)^2);
link_applied_torque = link_pivit_lenght*link_applied_force
torque_cog = cog*mass*9.81



left_over_torque = link_applied_torque - torque_cog


