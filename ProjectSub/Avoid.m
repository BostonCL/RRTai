function collision = Avoid(q1, q2, obstacles)
% Check if there is a collision between line segment q1-q2 and any obstacle

collision = true;

for i = 1:length(obstacles)
    obstacle = obstacles{i};
    for j = 1:size(obstacle,1)
        p1 = obstacle(j,:);
        if j == size(obstacle,1)
            p2 = obstacle(1,:);
        else
            p2 = obstacle(j+1,:);
        end
        if doLinesIntersect(q1, q2, p1, p2)
            collision = false;
            return
        end
    end
end

function intersect = doLinesIntersect(p1, q1, p2, q2)
% Check if two line segments intersect

intersect = false;

o1 = orientation(p1, q1, p2);
o2 = orientation(p1, q1, q2);
o3 = orientation(p2, q2, p1);
o4 = orientation(p2, q2, q1);

if o1 ~= o2 && o3 ~= o4
    intersect = true;
elseif o1 == 0 && onSegment(p1, p2, q1)
    intersect = true;
elseif o2 == 0 && onSegment(p1, q2, q1)
    intersect = true;
elseif o3 == 0 && onSegment(p2, p1, q2)
    intersect = true;
elseif o4 == 0 && onSegment(p2, q1, q2)
    intersect = true;
end

function o = orientation(p, q, r)
% Find orientation of ordered triplet (p,q,r)
% 0 --> p, q and r are colinear
% 1 --> Clockwise
% 2 --> Counterclockwise

val = (q(2) - p(2)) * (r(1) - q(1)) - (q(1) - p(1)) * (r(2) - q(2));

if val == 0
    o = 0;
elseif val > 0
    o = 1;
else
    o = 2;
end

function on = onSegment(p, q, r)
% Check if point q lies on line segment pr

on = false;

if q(1) <= max(p(1), r(1)) && q(1) >= min(p(1), r(1)) && ...
        q(2) <= max(p(2), r(2)) && q(2) >= min(p(2), r(2))
    on = true;
end