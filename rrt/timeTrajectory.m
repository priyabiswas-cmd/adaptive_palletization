function qTraj = timeTrajectory(waypoints)
%TIMETRAJECTORY Smooth, time-optimal motion through the waypoints (contopptraj).
%   qTraj = timeTrajectory(waypoints)
%     waypoints : K x 6 joint path (from planJointPath)
%     qTraj     : M x 6 joint angles, one row per controller step (0.02 s),
%                 respecting the velocity and acceleration limits in rrtConfig.

    rc = rrtConfig();
    if size(waypoints, 1) < 2
        qTraj = waypoints;
        return;
    end
    velLim = repmat([-1, 1] * rc.velocityLimit, 6, 1);
    accLim = repmat([-1, 1] * rc.accelerationLimit, 6, 1);

    [q, ~, ~, t] = contopptraj(waypoints', velLim, accLim, NumSamples = max(100, 2 * size(waypoints, 1)));

    % Sanity check: the motion can never need more than ~3x the time of
    % moving the largest joint distance at a gentle constant speed
    pathLength = sum(max(abs(diff(waypoints, 1, 1)), [], 2));
    maxReasonable = 3 * (pathLength / rc.velocityLimit + 2 * rc.velocityLimit / rc.accelerationLimit);
    if t(end) > maxReasonable
        error('timeTrajectory:badTiming', ...
              'contopptraj returned %.1f s for a path that should take at most %.1f s.', t(end), maxReasonable);
    end

    tSample = 0:rc.sampleTime:t(end);
    qTraj = interp1(t, q', tSample);
    qTraj(end+1, :) = waypoints(end, :);       % end exactly at the goal
end
