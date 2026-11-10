function env = environmentConfig()
%ENVIRONMENTCONFIG Geometry of the static work cell, in the world (= JSON) frame.
%   Origin on the floor directly under the robot base, z up, meters.
%   Every object is described by its centre and size [x y z] unless stated.

    % Floor
    env.floor.size  = [5, 5];
    env.floor.color = [81, 94, 120] / 255;

    % Platform the robot stands on (robot base is mounted at robotConfig().mountHeight)
    env.platform.centre = [0, 0, 0.25];
    env.platform.size   = [0.25, 0.25, 0.50];
    env.platform.color  = [87, 67, 60] / 255;

    % Static conveyor along x. Belt top surface at z = 0.50 (boxes rest on it).
    env.conveyor.centreXY  = [-0.60, 0.70];
    env.conveyor.length    = 1.10;          % along x
    env.conveyor.beltWidth = 0.40;          % along y
    env.conveyor.beltTop   = 0.50;
    env.conveyor.beltThickness = 0.02;
    env.conveyor.railHeight    = 0.10;      % side guide rails
    env.conveyor.railWidth     = 0.03;
    env.conveyor.rollerRadius  = 0.035;
    env.conveyor.legSize       = 0.05;
    env.conveyor.beltColor  = [0.10 0.10 0.10];
    env.conveyor.frameColor = [0.35 0.45 0.60];
    env.conveyor.legColor   = [0.30 0.30 0.32];
    env.conveyor.rollerColor = [0.70 0.70 0.72];

    % Euro pallet (mesh from the MathWorks example). Described by its
    % footprint in the world frame; top surface at z = 0.144.
    env.pallet.meshFile  = fullfile(fileparts(mfilename('fullpath')), 'assets', 'PalletEuro.stl');
    env.pallet.meshScale = 0.001;           % STL is in millimeters
    env.pallet.minCorner = [-1.20, -1.10, 0];
    env.pallet.size      = [0.80, 1.20, 0.144];
    env.pallet.color     = [0.72, 0.55, 0.36];   % light wood

    % Boxes
    env.box.color = [200, 100, 16] / 255;

    % Viewer camera: identical to the MathWorks example's Simulation 3D Scene
    % Configuration block (engine frame: 6 m behind, 2 m up, pitched 15 deg
    % down, 60 deg field of view, 1280 x 720 window).
    env.camera.translation = [-6, 0, 2];
    env.camera.rotation    = [0, -15, 0];   % value copied as-is from the example's block
    env.camera.fieldOfView = 60;
    env.camera.resolution  = [1280, 720];   % [width height]
end
