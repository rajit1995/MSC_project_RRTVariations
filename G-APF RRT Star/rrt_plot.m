function rrt_plot(RRTState)
    hold on
    fill([0, RRTState.Dimensions.Width, RRTState.Dimensions.Width, 0],...
        [0, 0, RRTState.Dimensions.Length, RRTState.Dimensions.Length],...
        [60, 125, 30]/255);
    scatter(RRTState.PointA(1), RRTState.PointA(2), 'filled', 'MarkerFaceColor', 'red');
    scatter(RRTState.PointB(1), RRTState.PointB(2), 'filled', 'MarkerFaceColor', 'green');
%     scatter(RRTState.pathvertices(:,1), RRTState.pathvertices(:,2), 'filled', 'MarkerFaceColor', 'm');

    
    for i = 1:RRTState.Obstacles.Number
        if i==3
            fill(RRTState.Obstacles.X(i,:), RRTState.Obstacles.Y(i,:), [20, 80, 160] / 255);
        else
            fill(RRTState.Obstacles.X(i,:), RRTState.Obstacles.Y(i,:), [68, 35, 0] / 255);
        end
        
        plot([RRTState.q_near(1), RRTState.Obstacles.Centers(i,1)], [RRTState.q_near(2), RRTState.Obstacles.Centers(i,2)],'--k');
    end
    plot([RRTState.q_near(1), RRTState.PointB(1)], [RRTState.q_near(2), RRTState.PointB(2)], '-k');
   for branchidx=1:size(RRTState.Branches1,1)      
        plot([RRTState.pathvertices(RRTState.Branches1(branchidx,1),1),RRTState.pathvertices(RRTState.Branches1(branchidx,2),1)],...
             [RRTState.pathvertices(RRTState.Branches1(branchidx,1),2),RRTState.pathvertices(RRTState.Branches1(branchidx,2),2)], 'b');

   end
   
   if RRTState.plotfinalpath ==1 
    for branchidx=1:size(RRTState.pathBranches,1)     
     plot([RRTState.pathvertices(RRTState.pathBranches(branchidx,1),1),RRTState.pathvertices(RRTState.pathBranches(branchidx,2),1)],...
    [RRTState.pathvertices(RRTState.pathBranches(branchidx,1),2),RRTState.pathvertices(RRTState.pathBranches(branchidx,2),2)],'r','LineWidth',2)
    end
   end

    scatter(RRTState.PointA(1), RRTState.PointA(2), 'filled', 'MarkerFaceColor', [0.5 0 0.8]);
    scatter(RRTState.PointB(1), RRTState.PointB(2), 'filled', 'MarkerFaceColor', 'green');
    axis equal;

    % Set plot limits based on field dimensions
    xlim([0, RRTState.Dimensions.Width]);
    ylim([0, RRTState.Dimensions.Length]);
    xlabel('x axis')
    ylabel('y axis')
    title('G-APF RRT Star Algorithm')

    hold off
  

end