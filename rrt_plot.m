function rrt_plot(RRTState)
    hold on
    fill([0, RRTState.Dimensions.Width, RRTState.Dimensions.Width, 0], [0, 0, RRTState.Dimensions.Length, RRTState.Dimensions.Length], RRTState.Terraincolour);
    scatter(RRTState.PointA(1), RRTState.PointA(2), 'filled', 'MarkerFaceColor', 'red');
    scatter(RRTState.PointB(1), RRTState.PointB(2), 'filled', 'MarkerFaceColor', 'green');
    %scatter(RRTState.q_new(1), RRTState.q_new(2), 'filled', 'MarkerFaceColor', 'm');
    scatter(RRTState.pathvertices(:,1), RRTState.pathvertices(:,2), 'filled', 'MarkerFaceColor', 'm');
    
    for i = 1:RRTState.Obstacles.Number
        fill(RRTState.Obstacles.X(i,:), RRTState.Obstacles.Y(i,:), 'k');
        plot([RRTState.q_near(1), RRTState.Obstacles.Centers(i,1)], [RRTState.q_near(2), RRTState.Obstacles.Centers(i,2)], '--b');
    end
    plot([RRTState.q_near(1), RRTState.PointB(1)], [RRTState.q_near(2), RRTState.PointB(2)], '--b');
    plot(RRTState.pathvertices(:,1), RRTState.pathvertices(:,2), 'b');
   
    axis equal;

    % Set plot limits based on field dimensions
    xlim([0, RRTState.Dimensions.Width]);
    ylim([0, RRTState.Dimensions.Length]);

    hold off

end