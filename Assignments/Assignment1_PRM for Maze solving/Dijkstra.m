function shortestPaths = Dijkstra(edges, milestones)
    % Creating the graph
    graph = containers.Map;
    for i = 1:size(edges, 1)
        startNode = [edges(i, 1), edges(i, 2)];
        endNode = [edges(i, 3), edges(i, 4)];
        
        if isKey(graph, startNode)
            graph(startNode) = [graph(startNode); endNode];
        else
            graph(startNode) = endNode;
        end
        % Uncomment the next block if the graph is undirected
        %{
        if isKey(graph, endNode)
            graph(endNode) = [graph(endNode); startNode];
        else
            graph(endNode) = startNode;
        end
        %}
    end

    % Dijkstra's algorithm
    shortestPaths = containers.Map;
    for i = 1:numel(milestones)
        startNode = [milestones{i}(1), milestones{i}(2)];
        unvisitedNodes = keys(graph);
        distances = inf(1, numel(unvisitedNodes));
        distances(ismember(cell2mat(unvisitedNodes), startNode, 'rows')) = 0;

        while ~isempty(unvisitedNodes)
            [minDistance, idx] = min(distances);
            currentNode = unvisitedNodes{idx};
            unvisitedNodes(idx) = [];

            if minDistance == inf
                break;
            end

            neighbors = graph(currentNode);
            for j = 1:numel(neighbors)
                neighborNode = neighbors{j};
                altDistance = minDistance + 1; % Assuming each edge has weight 1
                if altDistance < distances(ismember(cell2mat(unvisitedNodes), neighborNode, 'rows'))
                    distances(ismember(cell2mat(unvisitedNodes), neighborNode, 'rows')) = altDistance;
                end
            end
        end

        shortestPaths(milestones{i}) = distances;
    end
end
