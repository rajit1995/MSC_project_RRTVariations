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
Rover.n=2;
% Rover.LoSRadius = Rover.n*Rover.Length;
Rover.Radius = 0.5*norm([Rover.Length Rover.Width]);
Rover.LoSRadius = Rover.n*Rover.Length;
Rover.Dimensions = RRTState.Dimensions;
Rover.Obstacles = RRTState.Obstacles;
Rover.Kpu = 15;
Rover.Kiu = 30;
Rover.kdh = 0.05;
Rover.Kph = 20;
Rover.Kih = 0.1;
Rover.kdu = 0.01;
Rover.dt = 0.0002;
Rover.e_u_1 = [0 0];
Rover.u_sur_1 =[0 0];
Rover.waypoints = RRTState.finalpathvertices(:,1:2);
% Rover.pos_curr = Rover.waypoints(1,1:2);
Rover.pos_curr = Rover.waypoints(1,1:2);
Rover.pos_des =[0 0];
Rover.prev_wayPoint = Rover.waypoints(1,1:2);
Rover.Travel = [Rover.pos_curr];
Rover.counter = 0;
Rover.RadiusAcc = 0.4;
Rover.theta = 10;
Rover.theta_rad = Rover.theta*pi/180;
Rover.wpacc_ind = 0;
  for i = 2:size(Rover.waypoints,1)
      i
      if i==size(Rover.waypoints,1)
          Rover.RadiusAcc = 0.1;
          Rover.LoSRadius = norm(Rover.pos_curr - Rover.next_wayPoint)
      else 
          Rover.RadiusAcc = 0.7;
      end

     % for i = 2:3
     %     i
    Rover.next_wayPoint = Rover.waypoints(i,1:2); 
    prev_wayPoint  = Rover.prev_wayPoint
    next_wayPoint = Rover.next_wayPoint 
    pos_curr = Rover.pos_curr
    
    while Rover.wpacc_ind ~=1
     for j=1:36
       Rover.LosCircle(j,:) = [(Rover.pos_curr(1) + Rover.LoSRadius*cos(j*Rover.theta_rad)) (Rover.pos_curr(2) + Rover.LoSRadius*sin(j*Rover.theta_rad))];
       
    end
    loscircle = Rover.LosCircle;
    B = [Rover.prev_wayPoint(:,1),Rover.next_wayPoint(:,1)];
    C = [Rover.prev_wayPoint(:,2),Rover.next_wayPoint(:,2)];
    [xi,yi] = polyxpoly(B,C,Rover.LosCircle(:,1),Rover.LosCircle(:,2))
        if size(xi,1) > 1
            if norm([xi(1) yi(1)] - Rover.next_wayPoint) < norm([xi(2) yi(2)] - Rover.next_wayPoint)
                Rover.pos_des = [xi(1) yi(1)];
            else
                    Rover.pos_des = [xi(2) yi(2)];
            end
        elseif size(xi,1) == 1
            Rover.pos_des = [xi yi];
           pos_des1 = Rover.pos_des;
        else 
            Rover.pos_des = Rover.next_wayPoint;
        end
        pos_des = Rover.pos_des
        norma = norm(Rover.pos_curr - Rover.next_wayPoint)
        if norm(Rover.pos_curr - Rover.next_wayPoint) <= Rover.RadiusAcc 
             Rover.wpacc_ind = 1;
        elseif norm(Rover.pos_des - Rover.prev_wayPoint) < norm(Rover.pos_des - Rover.next_wayPoint) && i~= size(Rover.waypoints,1)
            Rover.wpacc_ind = 1;
        else
             Rover.wpacc_ind = 0;
        
         Rover.e_u = Rover.pos_des - Rover.pos_curr;
         u = Rover.e_u;
         Rover.u_sur = Rover.Kpu*Rover.e_u + Rover.Kiu*(Rover.e_u+Rover.e_u_1)*Rover.dt/2 + Rover.kdu*(Rover.e_u - Rover.e_u_1)/Rover.dt;
        Rover.a = (Rover.u_sur - Rover.u_sur_1)/Rover.dt;
         disp = Rover.u_sur_1*Rover.dt + 0.5*Rover.a*Rover.dt*Rover.dt;
        Rover.pos_curr = Rover.pos_curr + disp;
        pos_curr = Rover.pos_curr
        Rover.Travel = [Rover.Travel;Rover.pos_curr];
         Rover.e_u_1 = Rover.e_u;
         Rover.u_sur_1 = Rover.u_sur;
        end
    end
       Rover.prev_wayPoint = Rover.next_wayPoint;
       Rover.wpacc_ind = 0;
    
  end
  
    
