
function [rr, cc] = ray_trace(x1, y1, x2, y2)
    % Initialize the return arrays
    rr = [];
    cc = [];
    
    % Calculate the x and y distances between the two points
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    
    % Calculate the step size for the x and y axes
    if x1 < x2
        sx = 1;
    else
        sx = -1;
    end
    
    if y1 < y2
        sy = 1;
    else
        sy = -1;
    end
    
    % Initialize the error term
    err = dx - dy;
    
    % Traverse the line using Bresenham's algorithm
    while true
        % Add the current point to the return arrays
        rr = [rr; y1];
        cc = [cc; x1];
        
        % Check if we've reached the end point
        if x1 == x2 && y1 == y2
            break;
        end
        
        % Update the error term
        e2 = 2*err;
        if e2 > -dy
            err = err - dy;
            x1 = x1 + sx;
        end
        if e2 < dx
            err = err + dx;
            y1 = y1 + sy;
        end
    end
end

