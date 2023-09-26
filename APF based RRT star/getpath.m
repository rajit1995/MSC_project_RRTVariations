function RRTState = getpath(RRTState)
    numNodes = size(RRTState.pathvertices, 1);
    startNode = 1;
    for i = 1:size(RRTState.pathvertices, 1) 
        if RRTState.pathvertices(i,1:2) == RRTState.PointB
            endNode = i;
        end
    end

    tree = cell(1, numNodes);
    for i = 1:size(RRTState.Branches1, 1)
        parentIdx = RRTState.Branches1(i, 1);
        childIdx = RRTState.Branches1(i, 2);
        cost = RRTState.Branches1(i, 3); 
        
        tree{parentIdx} = [tree{parentIdx}; childIdx, cost];
        tree{childIdx} = [tree{childIdx}; parentIdx, cost]; % For undirected tree
    end

    % Check if start and end nodes are valid
    if startNode < 1 || startNode > numNodes || endNode < 1 || endNode > numNodes
        error('Invalid start or end node indices.');
    end

    % Initialize the queue and visited array
    queue = startNode;
    visited = false(1, numNodes);

    % Initialize parent array and branch details array to keep track of the path
    parent = zeros(1, numNodes);
    parent(startNode) = startNode;

    RRTState.branchDetailsArray = cell(1, numNodes); % Store branch details for each node
    RRTState.branchDetailsArray{startNode} = [];

    % Perform BFS traversal
    while ~isempty(queue)
        currentNode = queue(1);
        queue(1) = [];
        visited(currentNode) = true;

        % Check if the end node is reached
        if currentNode == endNode
            % Reconstruct the path from end to start
           currentNode = endNode;
           RRTState.path = currentNode;

            while currentNode ~= startNode
                currentNode = parent(currentNode);
                RRTState.path = [currentNode, RRTState.path];
            end
            RRTState.path;
    RRTState.pathBranches = [];
    for i = 1:length(RRTState.path)-1
        currentNode = RRTState.path(i);
        nextNode = RRTState.path(i+1);

        [~, idx] = ismember([currentNode, nextNode], RRTState.Branches1(:,1:2), 'rows');
        if idx > 0
            RRTState.pathBranches = [RRTState.pathBranches; RRTState.Branches1(idx, :)];
        else
            error('Error: Branch details not found for the path.');
        end
    end
            return;
        end

        % Add unvisited neighbors to the queue
        neighbours = tree{currentNode};
        for i = 1:size(neighbours, 1)
            neighbour = neighbours(i, 1);
            if ~visited(neighbour)
                queue(end+1) = neighbour;
                visited(neighbour) = true;
                parent(neighbour) = currentNode;
                RRTState.branchDetailsArray{neighbour} = [RRTState.branchDetailsArray{currentNode}; currentNode, neighbour];
            end
        end
    end

    end