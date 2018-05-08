function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

phi = state.rot(1,1);
theta = state.rot(2,1);
psi = state.rot(3,1);
psi_t = des_state.yaw;
psi_tdot = des_state.yawdot;
p = state.omega(1,1);
q = state.omega(2,1);
r = state.omega(3,1);
kd = [20; 20; 100];
kp = [12; 12; 50];
m = params.mass;
g = params.gravity;
kp_phi = 150;
kd_phi = 10;
kp_theta = 150;
kd_theta = 10;
kp_psi = 150;
kd_psi = 10;
p_des = 0;
q_des = 0;
r_des = psi_tdot;
r1_t_ddot = des_state.acc(1,1);
r2_t_ddot = des_state.acc(2,1);
r3_t_ddot = des_state.acc(3,1);
r_t_ddot = [r1_t_ddot; r2_t_ddot; r3_t_ddot];
r_t_dot = des_state.vel;
r_t = des_state.pos;
r_dot = state.vel;
r0 = state.pos;






% =================== Your code goes here ===================

% Thrust
% F = 0;
t_cap = des_state.acc/norm(des_state.acc);
n_cap = des_state.vel/norm(des_state.vel);
b_cap = cross(t_cap, n_cap);
e_v = r_t_dot - r_dot;
if(any(isnan(b_cap)))
    e_p = r_t - r0;
else
    e_p =(((r_t - r0)'*n_cap)*n_cap + ((r_t - r0)'*b_cap)*b_cap);
end
r_des_ddot = r_t_ddot + kd.*e_v + kp.*e_p;
r1_des_ddot = r_des_ddot(1,1);
r2_des_ddot = r_des_ddot(2,1);
r3_des_ddot = r_des_ddot(3,1);
u1 = m*(r3_des_ddot + g);
F = u1;

% Moment
% M = zeros(3,1);


phi_des = (r1_des_ddot*sin(psi_t) - r2_des_ddot*cos(psi_t))/g;
theta_des = (r1_des_ddot*cos(psi_t) + r2_des_ddot*sin(psi_t))/g;
psi_des = psi_t;

u2 = [kp_phi*(phi_des - phi) + kd_phi*(p_des - p);  
      kp_theta*(theta_des - theta) + kd_theta*(q_des - q); 
      kp_psi*(psi_des - psi) + kd_psi*(r_des - r);];
M = u2;
% =================== Your code ends here ===================

end
