function qTraj = verticalTrajectory(tree, cfg, qStart, fromTcp, toTcp, speed, dt)
%VERTICALTRAJECTORY Straight-line TCP move (e.g. 5 cm down to the box), cups facing down.
%   qTraj = verticalTrajectory(tree, cfg, qStart, fromTcp, toTcp, speed, dt)
%   The TCP follows the straight line fromTcp -> toTcp with a smooth
%   start and stop (peak speed = speed). One IK solution per step, each
%   chosen closest to the previous one, so the arm never changes shape.
%   qTraj : M x 6, one row per step of dt seconds; ends exactly at toTcp.

    distance = norm(toTcp - fromTcp);
    duration = 1.5 * distance / speed;          % cubic profile: peak = 1.5 x average speed
    n = max(2, ceil(duration / dt) + 1);
    u = linspace(0, 1, n)';
    s = 3*u.^2 - 2*u.^3;                        % smooth 0 -> 1, zero speed at both ends

    qTraj = zeros(n, 6);
    q = qStart;
    for i = 1:n
        p = fromTcp + s(i) * (toTcp - fromTcp);
        qNext = solveTcpIK(tree, cfg, p, q);
        if isempty(qNext)
            error('verticalTrajectory:unreachable', 'TCP position [%.3f %.3f %.3f] is not reachable.', p);
        end
        q = qNext;
        qTraj(i, :) = q;
    end
end
