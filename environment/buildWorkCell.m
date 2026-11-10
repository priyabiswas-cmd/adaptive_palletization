function buildWorkCell(World, Actor)
%BUILDWORKCELL Create the static work cell and the boxes inside the 3D world.
%   Called by the "Work Cell" Simulation 3D Actor block in palletizing.slx
%   (its initialization script is: buildWorkCell(World, Actor);).
%   Everything is placed from environmentConfig() and config/boxes.json, in
%   the world frame = JSON frame (origin on the floor under the robot base).

    env = environmentConfig();
    boxes = loadTaskPlan();

    % Floor
    floorActor = sim3d.Actor(ActorName = 'floor');
    createShape(floorActor, 'plane', [env.floor.size, 0.01]);
    floorActor.Color = env.floor.color;
    floorActor.PreciseContacts = true;
    add(World, floorActor, Actor);

    % Static equipment
    addStaticBox(World, Actor, 'platform', env.platform.centre, env.platform.size, env.platform.color);
    addConveyor(World, Actor, env.conveyor);
    addPallet(World, Actor, env.pallet);

    % Boxes (hidden until the task controller spawns them one by one)
    addBoxes(World, Actor, boxes, env.box.color);
end
