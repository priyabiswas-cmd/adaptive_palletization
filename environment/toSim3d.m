function [translation, rotation] = toSim3d(tform)
%TOSIM3D Convert a pose in our world (= JSON) frame to the 3D engine's frame.
%   [translation, rotation] = toSim3d(tform) for a 4x4 transform, or
%   translation = toSim3d(position) for a 1x3 position.
%
%   Our frame is right-handed (x, y, z up). The 3D engine (Unreal) is
%   left-handed: its y-axis points the other way. This is the ONLY place in
%   the project where that conversion happens; everything else uses our frame.

    if isequal(size(tform), [1, 3]) || isequal(size(tform), [3, 1])
        translation = sim3d.utils.Math.posToUnreal(tform(:)', 'matlab');
        rotation = [0, 0, 0];
        return;
    end
    translation = sim3d.utils.Math.posToUnreal(tform(1:3, 4)', 'matlab');
    eul = sim3d.utils.Math.decomp321(tform(1:3, 1:3));
    rotation = sim3d.utils.Math.rotToUnreal(eul, 'matlab');
end
