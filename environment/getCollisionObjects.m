function objects = getCollisionObjects(boxes, boxPositions, ignoreBoxIndex, margin)
%GETCOLLISIONOBJECTS Collision boxes of the work cell for the planner.
%   objects = getCollisionObjects(boxes, boxPositions, ignoreBoxIndex, margin)
%     boxes          : from loadTaskPlan()
%     boxPositions   : N x 3 current bottom-face centres of the boxes;
%                      a row of NaN = box has not appeared yet (ignored)
%     ignoreBoxIndex : box held by (or touched by) the gripper, left out; or []
%     margin         : optional safety margin (m) added on every side of every
%                      object except the platform the robot stands on
%   Returns a cell array of collisionBox objects in the world frame.

    if nargin < 3
        ignoreBoxIndex = [];
    end
    if nargin < 4
        margin = 0;
    end
    env = environmentConfig();
    objects = {};

    % Platform (no margin: the robot base is mounted 1 cm above it)
    objects{end+1} = makeBox(env.platform.centre, env.platform.size, 0);

    % Conveyor: belt block (top = belt surface, the boxes rest on it) plus
    % the two side rails, which stick out 2 cm above the belt
    c = env.conveyor;
    beltLength = c.length + 2 * c.rollerRadius;
    objects{end+1} = makeBox([c.centreXY, c.beltTop/2], [beltLength, c.beltWidth, c.beltTop], margin);
    railTop = c.beltTop + 0.02;
    railY = c.beltWidth/2 + c.railWidth/2;
    for side = [-1, 1]
        objects{end+1} = makeBox([c.centreXY(1), c.centreXY(2) + side*railY, railTop/2], ...
                                 [beltLength, c.railWidth, railTop], margin); %#ok<AGROW>
    end

    % Pallet
    p = env.pallet;
    objects{end+1} = makeBox(p.minCorner + p.size/2, p.size, margin);

    % Boxes that are in the world and not held by the gripper
    for k = 1:numel(boxes)
        if any(k == ignoreBoxIndex) || any(isnan(boxPositions(k, :)))
            continue;
        end
        b = boxes(k);
        objects{end+1} = makeBox(boxPositions(k, :) + [0, 0, b.h/2], [b.l, b.b, b.h], margin); %#ok<AGROW>
    end
end

function cb = makeBox(centre, sz, margin)
    cb = collisionBox(sz(1) + 2*margin, sz(2) + 2*margin, sz(3) + 2*margin);
    cb.Pose = trvec2tform(centre);
end
