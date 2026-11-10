function [ok, firstHit, hitInfo] = checkTrajectory(tree, cfg, qTraj, obstacles, heldBox)
%CHECKTRAJECTORY Collision check of every sample of a timed trajectory.
%   ok       : true if no sample collides
%   firstHit : index of the first colliding sample ([] if none)
%   hitInfo  : text naming what collides at firstHit ('' if none)

    rc = rrtConfig();
    planTree = attachHeldBox(tree, cfg, heldBox, rc.heldBoxMargin);
    ok = true;
    firstHit = [];
    hitInfo = '';
    for i = 1:size(qTraj, 1)
        inCollision = checkCollision(planTree, qTraj(i, :), obstacles, ...
            IgnoreSelfCollision = "off", SkippedSelfCollisions = "parent");
        if any(inCollision)
            ok = false;
            firstHit = i;
            hitInfo = describeHit(planTree, qTraj(i, :), obstacles);
            return;
        end
    end
end

function txt = describeHit(planTree, q, obstacles)
%DESCRIBEHIT Name the body/obstacle pairs in contact (for debugging).
    [~, sep] = checkCollision(planTree, q, obstacles, ...
        IgnoreSelfCollision = "off", SkippedSelfCollisions = "parent", Exhaustive = "on");
    names = [{planTree.Base.Name}, planTree.BodyNames];
    nBodies = numel(names);
    txt = '';
    [rows, cols] = find(isnan(sep));     % NaN separation = penetration
    for k = 1:numel(rows)
        a = names{min(rows(k), nBodies)};
        if cols(k) <= nBodies
            b = names{cols(k)};
        else
            b = sprintf('obstacle %d', cols(k) - nBodies);
        end
        txt = [txt, sprintf('%s<->%s; ', a, b)]; %#ok<AGROW>
    end
end
