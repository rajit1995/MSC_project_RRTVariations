function Rover = rovermodel(Rover)
for i = 2:size(Rover.waypoints,1)
     
    Rover.pos_des = Rover.waypoints(i,1:2);
    Rover.e_u = Rover.pos_des - Rover.pos_curr;
    while norm(Rover.e_u) > 0.001 
        Rover.counter = Rover.counter+1;
    Rover.e_u = Rover.pos_des - Rover.pos_curr;
    Rover.u_sur = Rover.Kpu*Rover.e_u + Rover.Kiu*(Rover.e_u+Rover.e_u_1)*Rover.dt/2 + Rover.kdu*(Rover.e_u - Rover.e_u_1)/Rover.dt;
    Rover.a = (Rover.u_sur-Rover.u_sur_1)/Rover.dt;
    disp = Rover.u_sur_1*Rover.dt + 0.5*Rover.a*Rover.dt*Rover.dt;

    Rover.pos_curr = Rover.pos_curr + disp;
    Rover.e_u_1 = Rover.e_u;
    Rover.Travel = [Rover.Travel;Rover.pos_curr];

    Rover.u_sur_1 = Rover.u_sur;
    visualrover(Rover);

    end


 end
 end