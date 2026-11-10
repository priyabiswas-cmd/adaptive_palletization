function addPallet(world, parent, p)
%ADDPALLET Load the Euro pallet mesh so its footprint matches p.minCorner / p.size (world frame).
%   p = environmentConfig().pallet

    pallet = sim3d.Actor(ActorName = 'pallet');
    add(world, pallet, parent);
    load(pallet, p.meshFile, p.meshScale * [1 1 1]);

    % The STL's own origin is one corner and it spans +x and +y in the
    % engine's frame. The engine's y is flipped, so the mesh corner sits at
    % our maximum y.
    pallet.Translation = toSim3d([p.minCorner(1), p.minCorner(2) + p.size(2), p.minCorner(3)]);

    % Wood look (the engine's default material is shiny metal)
    pallet.Color     = p.color;
    pallet.Metallic  = 0;
    pallet.Shininess = 0.1;
    pallet.Specular  = 0.1;
end
