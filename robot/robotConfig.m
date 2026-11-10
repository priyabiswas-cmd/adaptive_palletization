function cfg = robotConfig()
%ROBOTCONFIG Parameters of the UR10e cobot with the Robotiq EPick 4-cup vacuum gripper.
%   World frame = JSON frame: origin on the floor under the robot, z up.
%   Lengths in meters, angles in radians.

    % Raised platform the robot stands on (same as the MathWorks example)
    cfg.platform.size   = [0.25, 0.25, 0.50];   % [x y z]
    cfg.platform.centre = [0, 0, 0.25];
    cfg.mountHeight     = 0.51;                  % robot base_link height above the floor

    % Suction TCP: the face of the suction cups, measured along tool0 z.
    % tool0 -> 4cup_assembly is 0.114 m, cup faces are 0.076 m further.
    cfg.tcpOffset = 0.190;
    cfg.tcpBodyName = 'suction_tcp';

    % Orientation of the TCP for every pick and place: cups facing straight
    % down (TCP z = world -z), TCP x along world x.
    cfg.tcpRotationDown = [1  0  0;
                           0 -1  0;
                           0  0 -1];

    % Standard DH parameters of the UR10e (Universal Robots), used by the
    % closed-form IK solver. The toolbox model (loadrobot) equals this DH
    % model with its base rotated 180 deg about z; tool0 = DH flange.
    cfg.dh.a     = [0, -0.6127, -0.57155, 0, 0, 0];
    cfg.dh.d     = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655];
    cfg.dh.alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
    cfg.baseLinkToDh = [-1 0 0 0; 0 -1 0 0; 0 0 1 0; 0 0 0 1];   % Rz(pi)

    % Home configuration: TCP at [-0.55 0.10 0.95], cups facing down, between
    % the conveyor and the pallet. Elbow up, wrist down (same arm branch used
    % for every pick and place, so no flips during the task).
    cfg.homeConfig = [2.6449, -1.8344, 1.8793, 4.6674, -1.5708, 1.0741];
end
