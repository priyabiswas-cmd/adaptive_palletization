function setBoxPosition(world, box, bottomCentre)
%SETBOXPOSITION Move a box so its bottom-face centre is at bottomCentre (world frame).
%   world = sim3d.World.getWorld(modelName), box = one element of loadTaskPlan().

    actor = world.Actors.(boxActorName(box.id));
    actor.Translation = toSim3d(bottomCentre + [0, 0, box.h/2]);
end
