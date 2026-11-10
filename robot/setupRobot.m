function [tree, cfg] = setupRobot()
%SETUPROBOT Build the UR10e + platform + EPick vacuum gripper as one rigidBodyTree.
%   [tree, cfg] = setupRobot() returns the rigidBodyTree (row-vector joint
%   configurations) and the parameters from robotConfig().
%   Bodies of interest: 'tool0' (robot flange), '4cup_assembly' (gripper),
%   cfg.tcpBodyName (centre of the suction cup faces).

    cfg = robotConfig();

    tree = rigidBodyTree(MaxNumBodies = 15, DataFormat = "row");
    tree.BaseName = 'world';

    % Platform: the robot base sits on top of it
    platform = rigidBody('externalBase');
    platformJoint = rigidBodyJoint('externalBaseJoint', 'fixed');
    setFixedTransform(platformJoint, trvec2tform([0, 0, cfg.mountHeight]));
    platform.Joint = platformJoint;
    addBody(tree, platform, 'world');

    % UR10e arm
    ur10e = loadrobot("universalUR10e", DataFormat = "row");
    addSubtree(tree, 'externalBase', ur10e);

    % Robotiq EPick 4-cup vacuum gripper on the flange
    gripper = loadrobot("robotiqEPick4CupVacuumAssembly", DataFormat = "row");
    addSubtree(tree, 'tool0', gripper);

    % Suction TCP: centre of the cup faces, same orientation as tool0
    tcp = rigidBody(cfg.tcpBodyName);
    tcpJoint = rigidBodyJoint([cfg.tcpBodyName '_joint'], 'fixed');
    setFixedTransform(tcpJoint, trvec2tform([0, 0, cfg.tcpOffset]));
    tcp.Joint = tcpJoint;
    addBody(tree, tcp, 'tool0');

    % Joint limits (6x2), read once: reading tree.Bodies is slow
    cfg.jointLimits = zeros(6, 2);
    j = 0;
    for k = 1:tree.NumBodies
        joint = tree.Bodies{k}.Joint;
        if ~strcmp(joint.Type, 'fixed')
            j = j + 1;
            cfg.jointLimits(j, :) = joint.PositionLimits;
        end
    end
end
