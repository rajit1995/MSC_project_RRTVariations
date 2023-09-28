function RRTState = findShortestPath(RRTState)
% vertices, branches, startNode, endNode
    numNodes = size(RRTState.pathvertices, 1);
    
    % Convert the branches data structure to an adjacency list representation
    adjacencyList = cell(numNodes, 1);
    for i = 1:size(RRTState.Branches1, 1)
        parent = RRTState.Branches1(i, 1);
        child = RRTState.Branches1(i, 2);
        length = RRTState.Branches1(i, 3);
        adjacencyList{parent} = [adjacencyList{parent}; child, length];
        adjacencyList{child} = [adjacencyList{child}; parent, length];
    end
    
    % Helper function to calculate the Euclidean distance between two nodes
    function dist = distance(node1, node2)
        x1 = RRTState.pathvertices(node1, 1);
        y1 = RRTState.pathvertices(node1, 2);
        x2 = RRTState.pathvertices(node2, 1);
        y2 = RRTState.pathvertices(node2, 2);
        dist = sqrt((x1 - x2)^2 + (y1 - y2)^2);
    end

    % Initialize variables for Dijkstra's algorithm
    visited = false(numNodes, 1);
    shortestDistances = Inf(numNodes, 1);
    previousNodes = zeros(numNodes, 1);

    shortestDistances(RRTState.PointA) = 0;

    % Dijkstra's algorithm
    for i = 1:numNodes
        currentDistances = shortestDistances;
        currentDistances(visited) = Inf;
        [~, currentNode] = min(currentDistances);

        if isinf(currentDistances(currentNode))
            break;
        end

        visited(currentNode) = true;

        neighbors = adjacencyList{currentNode};
        for j = 1:size(neighbors, 1)
            neighbor = neighbors(j, 1);
            edgeWeight = neighbors(j, 2);
            altDistance = shortestDistances(currentNode) + edgeWeight;

            if altDistance < shortestDistances(neighbor)
                shortestDistances(neighbor) = altDistance;
                previousNodes(neighbor) = currentNode;
            end
        end
    end

    % Construct the shortest path from startNode to endNode
    path = RRTState.PointB;
    while path(1) ~= RRTState.PointA
        path = [previousNodes(path(1)), path];
    end

    RRT.shortestPath = path;
end