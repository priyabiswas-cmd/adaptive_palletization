function [waypoints, info] = planJointPath(tree, cfg, qStart, qGoal, obstacles, heldBox)
%PLANJOINTPATH Collision-free joint-space path with manipulatorRRT.
%   [waypoints, info] = planJointPath(tree, cfg, qStart, qGoal, obstacles, heldBox)
%     tree, cfg  : from setupRobot()
%     qStart, qGoal : 1x6 joint angles
%     obstacles  : cell array of collision objects, normally
%                  getCollisionObjects(..., rrtConfig().obstacleMargin)
%     heldBox    : box carried by the gripper (one element of loadTaskPlan), or []
%     waypoints  : K x 6 dense, shortened path from qStart to qGoal
%     info       : struct with planningTime, numRawWaypoints, numWaypoints

    rc = rrtConfig();
    planTree = attachHeldBox(tree, cfg, heldBox, rc.heldBoxMargin);

    % Start and goal must themselves be collision-free
    assertFree(planTree, qStart, obstacles, 'start');
    assertFree(planTree, qGoal,  obstacles, 'goal');

    planner = manipulatorRRT(planTree, obstacles);
    planner.IgnoreSelfCollision   = false;
    planner.SkippedSelfCollisions = "parent";
    planner.MaxConnectionDistance = rc.maxConnectionDistance;
    planner.ValidationDistance    = rc.validationDistance;
    planner.MaxIterations         = rc.maxIterations;

    previousSeed = rng(rc.randomSeed);
    restoreSeed = onCleanup(@() rng(previousSeed));

    tStart = tic;
    [rawPath, solution] = plan(planner, qStart, qGoal);
    if ~solution.IsPathFound
        error('planJointPath:noPath', 'RRT found no path after %d iterations.', solution.NumIterations);
    end
    shortPath = shorten(planner, rawPath, rc.shortenIterations);
    info.planningTime = toc(tStart);

    % Dense waypoints along the shortened (collision-checked) segments
    waypoints = densify(shortPath, rc.interpolationStep);
    info.numRawWaypoints = size(rawPath, 1);
    info.numWaypoints = size(shortPath, 1);
end

function assertFree(planTree, q, obstacles, label)
    [inCollision, sepDist] = checkCollision(planTree, q, obstacles, ...
        IgnoreSelfCollision = "off", SkippedSelfCollisions = "parent", Exhaustive = "on");
    if any(inCollision)
        error('planJointPath:inCollision', 'The %s configuration is in collision (min distance %.3f m).', ...
              label, min(sepDist(:), [], 'omitnan'));
    end
end

function out = densify(path, step)
%DENSIFY Waypoints every `step` rad along the path. (Near-)repeated points
%   are dropped first: contopptraj cannot time tiny steps (it returns
%   absurdly long motions).
    tol = 1e-3;                                 % rad
    goal = path(end, :);
    clean = path(1, :);
    for i = 2:size(path, 1)
        if max(abs(path(i, :) - clean(end, :))) >= tol
            clean(end+1, :) = path(i, :); %#ok<AGROW>
        end
    end
    clean(end, :) = goal;                       % always end exactly at the goal
    path = clean;
    out = path(1, :);
    for i = 2:size(path, 1)
        n = max(1, ceil(max(abs(path(i, :) - path(i-1, :))) / step));
        s = (1:n)' / n;
        out = [out; path(i-1, :) + s .* (path(i, :) - path(i-1, :))]; %#ok<AGROW>
    end
end
