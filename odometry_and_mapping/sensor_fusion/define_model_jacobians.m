function define_model_jacobians()

% Current state
x = sym('x',[28 1]);
assume(x, 'rational');
% Current visual odometry measurement
z_vo = sym('z_vo',[7 1]);
assume(z_vo, 'rational');

% MOTION MODEL
tic; x_kp1 = g_fnc(x, z_vo); size(x_kp1), toc


%%%% dg/dx
tic; disp('dgdx')
symbolic = jacobian(x_kp1, x'); toc
tic; fcn = symfun(symbolic, [x; z_vo]'); toc
tic; matlabFunction(fcn,'File', 'dgdx');%,'Optimize',false); toc
toc


%%%% dg/d(z_vo)
tic; disp('dgdzvo')
symbolic = jacobian(x_kp1, z_vo'); toc
tic; fcn = symfun(symbolic, [x; z_vo]'); toc
tic; matlabFunction(fcn,'File', 'dgdzVO');%,'Optimize',false); toc
toc

%%

% MEASUREMENT MODEL
joint_visible = 1;
in_bend = 1;
new_bend = 1;
tic; z = h_fnc(x, in_bend, joint_visible, new_bend); size(z), toc
tic; z = simplify(z); toc

%%%% dh/dx components
tic; disp('J_CF')
symbolic = jacobian(z(1:7), x'); toc
tic; fcn = symfun(symbolic, x'); toc
tic; matlabFunction(fcn,'File', 'J_CF');%,'Optimize',false); toc
toc

tic; disp('J_VJT')
symbolic = jacobian(z(8:10), x'); toc
tic; fcn = symfun(symbolic, x'); toc
tic; matlabFunction(fcn,'File', 'J_VJT');%,'Optimize',false); toc
toc

tic; disp('J_PE_R_CF')
symbolic = jacobian(z(11:14), x'); toc
tic; fcn = symfun(symbolic, x'); toc
tic; matlabFunction(fcn,'File', 'J_PE_R_CF');%,'Optimize',false); toc
toc

tic; disp('J_PE_pipe')
symbolic = jacobian(z(15:18), x'); toc
tic; fcn = symfun(symbolic, x'); toc
tic; matlabFunction(fcn,'File', 'J_PE_pipe');%,'Optimize',false); toc
toc

tic; disp('J_PE_t_CF')
symbolic = jacobian(z(19:21), x'); toc
tic; fcn = symfun(symbolic, x'); toc
tic; matlabFunction(fcn,'File', 'J_PE_t_CF');%,'Optimize',false); toc
toc

tic; disp('J_PE_t_pipe')
symbolic = jacobian(z(22:24), x'); toc
tic; fcn = symfun(symbolic, x'); toc
tic; matlabFunction(fcn,'File', 'J_PE_t_pipe');%,'Optimize',false); toc
toc

return











































