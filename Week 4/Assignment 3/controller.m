function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

%u1 = 0;
%u2 = 0;

% FILL IN YOUR CODE HERE

m = params.mass;
g = params.gravity;
kv_z =  12;
kp_z = 65;
kv_phi = 120;
kp_phi = 1300;
kv_y = 10;
kp_y = 50;
zdes_ddot = des_state.acc(2,1);
zdes_dot = des_state.vel(2,1);
zdes = des_state.pos(2,1);
z_dot = state.vel(2,1);
z = state.pos(2,1);
Ixx = params.Ixx;
ydes_ddot = des_state.acc(1,1);
ydes_dot = des_state.vel(1,1);
ydes = des_state.pos(1,1);
y_dot = state.vel(1,1);
y = state.pos(1,1);
phi_dot = state.omega;
phi = state.rot;
phic_dot = 0;
phic_ddot = 0;


u1 = m*(g + zdes_ddot + kv_z*(zdes_dot - z_dot) + kp_z*(zdes - z));

phic = -(ydes_ddot + kv_y*(ydes_dot - y_dot) + kp_y*(ydes - y))/g;

phides_ddot = (phic_ddot + kv_phi*(phic_dot - phi_dot) + kp_phi*(phic - phi));

u2 = Ixx*phides_ddot;


end

