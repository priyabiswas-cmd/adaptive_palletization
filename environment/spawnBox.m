function spawnBox(world, box)
%SPAWNBOX Make a box appear at its JSON start position (on the conveyor).
%   world = sim3d.World.getWorld(modelName), box = one element of loadTaskPlan().

    actor = world.Actors.(boxActorName(box.id));
    actor.Translation = toSim3d(box.start + [0, 0, box.h/2]);
    actor.Hidden = false;
end
