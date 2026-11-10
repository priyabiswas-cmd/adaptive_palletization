function addConveyor(world, parent, c)
%ADDCONVEYOR Build the static conveyor from simple parts: belt, side rails, end rollers, legs.
%   c = environmentConfig().conveyor. The belt does not move.

    cx = c.centreXY(1);
    cy = c.centreXY(2);
    L  = c.length;
    W  = c.beltWidth;

    % Belt: top surface exactly at c.beltTop
    addStaticBox(world, parent, 'conveyorBelt', ...
        [cx, cy, c.beltTop - c.beltThickness/2], [L, W, c.beltThickness], c.beltColor);

    % Side frame / guide rails
    railZ = c.beltTop - c.railHeight/2 + 0.02;      % rails stick out 2 cm above the belt
    railY = W/2 + c.railWidth/2;
    addStaticBox(world, parent, 'conveyorRailLeft',  [cx, cy + railY, railZ], [L, c.railWidth, c.railHeight], c.frameColor);
    addStaticBox(world, parent, 'conveyorRailRight', [cx, cy - railY, railZ], [L, c.railWidth, c.railHeight], c.frameColor);

    % End rollers (cylinders along y)
    rollerZ = c.beltTop - c.rollerRadius;
    for side = [-1, 1]
        roller = sim3d.Actor(ActorName = sprintf('conveyorRoller%d', (side + 3) / 2));
        createShape(roller, 'cylinder', [2*c.rollerRadius, 2*c.rollerRadius, W]);
        T = trvec2tform([cx + side * L/2, cy, rollerZ]) * axang2tform([1 0 0 pi/2]);
        [roller.Translation, roller.Rotation] = toSim3d(T);
        roller.Color = c.rollerColor;
        roller.Metallic = 0.8;
        add(world, roller, parent);
    end

    % Lower cross beam and four legs
    frameBottom = c.beltTop - c.railHeight;
    addStaticBox(world, parent, 'conveyorBeam', [cx, cy, frameBottom - 0.02], [L - 0.1, W, 0.04], c.frameColor);
    legHeight = frameBottom;
    k = 0;
    for sx = [-1, 1]
        for sy = [-1, 1]
            k = k + 1;
            pos = [cx + sx * (L/2 - 0.08), cy + sy * (W/2 - c.legSize/2), legHeight/2];
            addStaticBox(world, parent, sprintf('conveyorLeg%d', k), pos, [c.legSize, c.legSize, legHeight], c.legColor);
        end
    end
end
