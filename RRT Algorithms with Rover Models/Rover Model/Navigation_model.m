clc;
clearvars;
rrtvariables;
load("FieldConst.mat");
load("RRTState.mat");
%%
Rover.PointA = RRTState.PointA;
Rover.PointB = RRTState.PointB;
Rover.Length = 0.35;
Rover.Width = 0.2488;
Rover.Radius = 0.5*norm([Rover.Length Rover.Width]);
Rover.Dimensions = RRTState.Dimensions;
Rover.Obstacles = RRTState.Obstacles;
Rover.Kpu = 15;
Rover.Kiu = 30;
Rover.kdh = 0.05;
Rover.Kph = 20;
Rover.Kih = 0.1;
Rover.kdu = 0.01;
Rover.dt = 0.02;
Rover.e_u_1 = [0 0];
Rover.u_sur_1 =[0 0];
Rover.waypoints = RRTState.finalpathvertices(:,1:2);
Rover.pos_curr = Rover.waypoints(1,1:2);
Rover.Travel = [Rover.pos_curr];
Rover.counter = 0;
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