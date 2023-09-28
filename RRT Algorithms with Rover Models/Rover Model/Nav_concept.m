clc
pointA = [1 1];
pointB = [3 4];
Kpu = 15;
Kiu = 30;
kdh = 0.05;
Kph = 20;
Kih = 0.1;
kdu = 0.01;
dt = 0.001;
d_curr = pointA;
d_des  = pointB;
psi_curr = 0;
e_u = d_des - d_curr;
psi_des = atan2(e_u(2),e_u(1));
e_psi = psi_des - psi_curr;
e_psi_1 = psi_des - psi_curr;
e_u_1 = [0 0];
u_sur_1 =0;

while norm(e_u) > 0.01 
    norm(e_u)
    e_u = d_des - d_curr;
    % psi_des = atan2(e_u(2),e_u(1));
    % e_psi = psi_des - psi_curr;
    u_sur = Kpu*e_u + Kiu*(e_u+e_u_1)*dt/2 + kdu*(e_u - e_u_1)/dt;
    % u_head = Kph*e_psi + Kih*e_psi*dt + kdh*(e_psi - e_psi_1)/dt;
    % V1 = (u_sur + u_head)/2  ;
    % v2 = (u_sur - u_head)/2 ;
    % v = v1-v2;
    a = (u_sur-u_sur_1)/dt
    disp = u_sur_1*dt + 0.5*a*dt*dt;
    
    d_curr = d_curr + disp;
    e_u_1 = e_u;
    e_psi_1 = e_psi;
    u_sur_1 = u_sur;
    hold on

    scatter(pointA(1), pointA(2), 'filled', 'MarkerFaceColor', 'red');
    scatter(pointB(1), pointB(2), 'filled', 'MarkerFaceColor', 'green');
    scatter(d_curr(1), d_curr(2), 'filled', 'MarkerFaceColor', 'blue');
    hold off
    norm(e_u);
end