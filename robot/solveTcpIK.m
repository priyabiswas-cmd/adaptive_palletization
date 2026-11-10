function [q, allSolutions] = solveTcpIK(tree, cfg, tcpPosition, qRef)
%SOLVETCPIK Joint angles that put the suction TCP at tcpPosition, cups facing down.
%   [q, allSolutions] = solveTcpIK(tree, cfg, tcpPosition, qRef)
%     tree, cfg   : from setupRobot()
%     tcpPosition : [x y z] of the suction-cup face centre, world frame
%     qRef        : 1x6 joint angles to stay close to (e.g. the current config)
%     q           : valid solution closest to qRef, or [] if unreachable
%     allSolutions: every valid solution (k x 6)
%
%   Each solution is confirmed with the toolbox forward kinematics
%   (getTransform), and must lie inside the toolbox joint limits.

    % Desired TCP pose in the world
    Ttcp = eye(4);
    Ttcp(1:3, 1:3) = cfg.tcpRotationDown;
    Ttcp(1:3, 4) = tcpPosition(:);

    % TCP -> tool0 (flange) -> robot base_link -> DH base frame
    Ttool0 = Ttcp * trvec2tform([0, 0, -cfg.tcpOffset]);
    TbaseLink = trvec2tform([0, 0, cfg.mountHeight]);
    Tdh = cfg.baseLinkToDh \ (TbaseLink \ Ttool0);

    % Up to 8 arm branches. Confirm each with the toolbox model, then shift
    % every joint by +-2*pi to the copy closest to qRef (same pose).
    candidates = urClosedFormIK(cfg.dh, Tdh);
    candidates = confirmWithToolbox(tree, cfg, candidates, Ttcp);
    limits = cfg.jointLimits;
    allSolutions = zeros(0, 6);
    for k = 1:size(candidates, 1)
        qk = closestInLimits(candidates(k, :), qRef, limits);
        if ~isempty(qk)
            allSolutions(end+1, :) = qk; %#ok<AGROW>
        end
    end

    if isempty(allSolutions)
        q = [];
        return;
    end
    [~, best] = min(sum((allSolutions - qRef).^2, 2));
    q = allSolutions(best, :);
end

function q = closestInLimits(q, qRef, limits)
%CLOSESTINLIMITS For each joint pick q(j) + n*2*pi nearest to qRef(j) inside the limits.
    for j = 1:6
        v = q(j) + 2*pi * (-2:2);
        v = v(v >= limits(j, 1) - 1e-9 & v <= limits(j, 2) + 1e-9);
        if isempty(v)
            q = [];
            return;
        end
        [~, i] = min(abs(v - qRef(j)));
        q(j) = v(i);
    end
end

function valid = confirmWithToolbox(tree, cfg, sols, Ttarget)
%CONFIRMWITHTOOLBOX Keep solutions whose toolbox FK reaches the target (within 10 micrometers).
    tol = 1e-5;
    valid = zeros(0, 6);
    for k = 1:size(sols, 1)
        T = getTransform(tree, sols(k, :), cfg.tcpBodyName);
        if norm(T(1:3, 4) - Ttarget(1:3, 4)) < tol && norm(T(1:3, 1:3) - Ttarget(1:3, 1:3), 'fro') < tol
            valid(end+1, :) = sols(k, :); %#ok<AGROW>
        end
    end
end
