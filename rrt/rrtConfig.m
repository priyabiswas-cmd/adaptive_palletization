function c = rrtConfig()
%RRTCONFIG Settings for the RRT planner and the motion timing.

    % manipulatorRRT (same values as the MathWorks example)
    c.maxConnectionDistance = 0.3;    % rad, longest edge added to the tree
    c.validationDistance    = 0.05;   % rad, collision-check spacing along edges
    c.shortenIterations     = 20;     % shortcutting passes after a path is found
    c.maxIterations         = 10000;
    c.randomSeed            = 0;      % repeatable plans

    % Waypoint density before timing: a smooth curve through dense waypoints
    % stays close to the collision-checked straight segments
    c.interpolationStep = 0.05;       % rad between waypoints

    % Motion timing (contopptraj, as in the example). Slower than the
    % example's +-10 rad/s so the motion is easy to follow.
    c.velocityLimit     = 1.0;        % rad/s, every joint
    c.accelerationLimit = 2.0;        % rad/s^2, every joint
    c.sampleTime        = 0.02;       % s, controller step in palletizing.slx

    % Held box: collision shape is the box grown by this margin on its sides
    c.heldBoxMargin = 0.01;           % m

    % Safety margin around obstacles during RRT planning only, so the smooth
    % timed curve may deviate slightly from the checked path and stay clear
    c.obstacleMargin = 0.02;          % m
end
