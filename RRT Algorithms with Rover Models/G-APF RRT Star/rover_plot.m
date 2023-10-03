function rover_plot(Rover)
    hold on
    fill([0, Rover.Dimensions.Width, Rover.Dimensions.Width, 0],...
        [0, 0, Rover.Dimensions.Length, Rover.Dimensions.Length],...
        [60, 125, 30]/255);
    scatter(Rover.PointA(1), Rover.PointA(2), 'filled', 'MarkerFaceColor', 'red');
    scatter(Rover.PointB(1), Rover.PointB(2), 'filled', 'MarkerFaceColor', 'green');
%     scatter(RRTState.pathvertices(:,1), RRTState.pathvertices(:,2), 'filled', 'MarkerFaceColor', 'm');

    
    for i = 1:Rover.Obstacles.Number
        if i==3
            fill(Rover.Obstacles.X(i,:), Rover.Obstacles.Y(i,:), [20, 80, 160] / 255);
        else
            fill(Rover.Obstacles.X(i,:), Rover.Obstacles.Y(i,:), [68, 35, 0] / 255);
        end

    end
    for branchidx=1:size(Rover.Travel,1)     
     plot([Rover.Travel(:,1)],  [Rover.Travel(:,2)],'r','LineWidth',2)
    end
  
    for branchidx=1:size(Rover.waypoints,1)     
     plot([Rover.waypoints(:,1)],  [Rover.waypoints(:,2)],'k--','LineWidth',1)
    end
   
    % r = Rover.Radius;
    % pos = Rover.pos_curr;
      rectangle('Position',[Rover.pos_curr(1)-Rover.Radius, Rover.pos_curr(2)-Rover.Radius, 2*Rover.Radius, 2*Rover.Radius],'Curvature', [1,1], 'FaceColor','w')
        dir = Rover.u_sur/norm(Rover.u_sur);
    plot([Rover.pos_curr(1),Rover.pos_curr(1)+dir(1)*Rover.Radius],[Rover.pos_curr(2), Rover.pos_curr(2)+dir(2)*Rover.Radius],'LineWidth', 0.3,'Color','k');
    
    % sz = 100
    % scatter(Rover.pos_curr(1), Rover.pos_curr(2),sz,'square', 'filled', 'MarkerFaceColor', 'blue');
   

   
    axis equal;

    % Set plot limits based on field dimensions
    xlim([0, Rover.Dimensions.Width]);
    ylim([0, Rover.Dimensions.Length]);
    xlabel('x axis')
    ylabel('y axis')
    title('Rover model')
    hold off
  
end