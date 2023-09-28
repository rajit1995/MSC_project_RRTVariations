% Set the dimensions of the field
load("Obstacle_data.mat")
%fieldWidth = 25; % Width of the field in meters
%fieldHeight = 25; % Height of the field in meters
%K_att = 3
%K_epd = 2


% Set the color for the inside of the field
colorInside = [0.8, 0.6, 0.4]; % RGB values for light brown color

% Set the coordinates for point A and point B
%pointA = [2, 3]; % Coordinates of point A
%pointB = [20, 23]; % Coordinates of point B

% Create a figure
figure;
hold on;

% Plot the field
fill([0, fieldWidth, fieldWidth, 0], [0, 0, fieldHeight, fieldHeight], colorInside);

% Plot point A and point B
scatter(pointA(1), pointA(2), 'filled', 'MarkerFaceColor', 'red');
scatter(pointB(1), pointB(2), 'filled', 'MarkerFaceColor', 'green');
% Define the number of obstacles
numObstacles = 5;


% Generate and plot obstacles
for i = 1:numObstacles
    fill(obstacleX(i,:), obstacleY(i,:), 'k');
end

plot([pointA(1), pointB(1)], [pointA(2), pointB(2)], 'b');




random_q = [rand() * fieldWidth,rand() * fieldHeight];
% Plot the random point
scatter(random_q(1), random_q(2), 'filled', 'MarkerFaceColor', 'yellow');

% Plot lines connecting point A to point B and point A to the random point
plot([pointA(1), pointB(1)], [pointA(2), pointB(2)], 'b');
plot([pointA(1), random_q(1)], [pointA(2), random_q(2)], 'r');


e_nr = (random_q - pointA)/norm((random_q - pointA));
e_ng = (pointB - pointA)/norm((pointB - pointA));
F_gra = K_att*e_nr;
Expand_fun = K_epd*e_ng;
Growth = F_gra + Expand_fun;
q_new = pointA + Growth;
scatter(q_new(1), q_new(2), 'filled', 'MarkerFaceColor', 'black');
axis equal;

% Set plot limits based on field dimensions
xlim([0, fieldWidth]);
ylim([0, fieldHeight]);

% Remove extra whitespace around the plot
box on;
set(gca, 'Layer', 'top');

hold off;

