function actor = addStaticBox(world, parent, name, centre, sz, color)
%ADDSTATICBOX Add a solid, non-moving box to the 3D world.
%   centre = [x y z] of the box centre in our world frame, sz = [x y z] size.
%   parent = actor to attach it to (the work cell actor).

    actor = sim3d.Actor(ActorName = name);
    createShape(actor, 'box', sz);
    actor.Translation = toSim3d(centre);
    actor.Color = color;
    actor.Shininess = 0.3;
    actor.Metallic = 0.2;
    add(world, actor, parent);
end
