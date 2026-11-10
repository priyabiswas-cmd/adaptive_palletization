function addBoxes(world, parent, boxes, color)
%ADDBOXES Create one box actor per JSON box, hidden until spawnBox() shows it.
%   Actor names are 'box<id>', e.g. 'box1'. Every box waits (hidden) at its
%   start position; the task controller makes them appear one after another,
%   like the MathWorks example.

    for k = 1:numel(boxes)
        b = boxes(k);
        actor = sim3d.Actor(ActorName = boxActorName(b.id), ...
                            Mobility = sim3d.utils.MobilityTypes.Movable);
        createShape(actor, 'box', [b.l, b.b, b.h]);
        actor.Translation = toSim3d(b.start + [0, 0, b.h/2]);   % shape origin = box centre
        actor.Color = color;
        actor.Shininess = 0.2;
        actor.Metallic = 0;
        actor.Mass = b.weight;          % same weight as in the JSON
        actor.Physics = false;          % moved by the gripper, not by physics
        actor.Hidden = true;            % appears when spawned
        add(world, actor, parent);
    end
end
