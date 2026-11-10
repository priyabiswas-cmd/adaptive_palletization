classdef PalletizingStateMachine < handle
%PALLETIZINGSTATEMACHINE Pick-and-place task for all boxes in the JSON plan.
%
%   For every box k:
%     MOVE_TO_PREPICK      RRT ON : current pose -> 5 cm above the box top
%     DESCEND_TO_PICK      RRT OFF: straight down to the box top (z + h)
%     GRASP                suction on, box attached to the gripper
%     ASCEND_TO_POSTPICK   straight up 5 cm with the box
%     MOVE_TO_PREPLACE     RRT ON : carry the box to 5 cm above its goal
%     DESCEND_TO_PLACE     RRT OFF: straight down until the box rests on its goal
%     RELEASE              suction off, box left at the goal; next box appears
%     ASCEND_TO_POSTPLACE  straight up 5 cm
%   then the next box; after the last one:
%     MOVE_TO_HOME         RRT ON : back to the home configuration
%     DONE
%   If anything cannot be planned the machine goes to FAILED and holds still.
%
%   Usage:
%     sm = PalletizingStateMachine(tree, cfg, boxes, env);
%     q  = sm.step();          % call once per controller step (0.02 s)
%   env is a struct of two function handles provided by the simulator:
%     env.spawnBox(k)                  make box k appear at its start
%     env.moveBox(k, bottomCentre)     put box k at a position (world frame)

    properties (SetAccess = private)
        State        = "START"   % current state name
        BoxIndex     = 1         % box being handled (index into the plan)
        Q                        % current joint angles (1x6)
        Time         = 0         % simulation time (s)
        BoxPositions             % N x 3 bottom-face centres (NaN = not appeared yet)
    end

    properties (Access = private)
        Tree
        Cfg
        Boxes
        Env
        Settings                 % stateMachineConfig()
        HeldBox      = 0         % index of the box on the gripper, 0 = none
        Trajectory               % joint angles still to play (M x 6)
        TrajIndex    = 1
        WaitSteps    = 0
    end

    methods
        function obj = PalletizingStateMachine(tree, cfg, boxes, env)
            obj.Tree = tree;
            obj.Cfg = cfg;
            obj.Boxes = boxes;
            obj.Env = env;
            obj.Settings = stateMachineConfig();
            obj.Q = cfg.homeConfig;
            obj.BoxPositions = nan(numel(boxes), 3);
        end

        function q = step(obj)
        %STEP Advance the task by one controller step and return the joint angles.
            switch obj.State
                case "START"
                    obj.log('TASK', 'Starting: %d boxes to palletize.', numel(obj.Boxes));
                    obj.spawn(1);
                    obj.enter("MOVE_TO_PREPICK");

                case {"MOVE_TO_PREPICK", "DESCEND_TO_PICK", "ASCEND_TO_POSTPICK", ...
                      "MOVE_TO_PREPLACE", "DESCEND_TO_PLACE", "ASCEND_TO_POSTPLACE", "MOVE_TO_HOME"}
                    obj.Q = obj.Trajectory(obj.TrajIndex, :);
                    obj.TrajIndex = obj.TrajIndex + 1;
                    if obj.TrajIndex > size(obj.Trajectory, 1)
                        obj.arrived();
                    end

                case {"GRASP", "RELEASE"}
                    obj.WaitSteps = obj.WaitSteps - 1;
                    if obj.WaitSteps <= 0
                        obj.arrived();
                    end

                otherwise   % DONE, FAILED: hold still
            end

            % A held box follows the suction cups: its top face stays at the TCP
            if obj.HeldBox > 0
                b = obj.Boxes(obj.HeldBox);
                obj.setBox(obj.HeldBox, obj.tcpPosition() - [0, 0, b.h]);
            end

            obj.Time = obj.Time + obj.Settings.sampleTime;
            q = obj.Q;
        end

        function tf = isDone(obj)
            tf = any(obj.State == ["DONE", "FAILED"]);
        end
    end

    methods (Access = private)
        % ---------------- transitions ----------------
        function arrived(obj)
        %ARRIVED The current state's motion or wait is finished: go to the next state.
            k = obj.BoxIndex;
            switch obj.State
                case "MOVE_TO_PREPICK"
                    obj.log('RRT', 'OFF: reached PRE-PICK of box %d (%s).', obj.Boxes(k).id, obj.tcpText());
                    obj.enter("DESCEND_TO_PICK");
                case "DESCEND_TO_PICK"
                    obj.enter("GRASP");
                case "GRASP"
                    obj.enter("ASCEND_TO_POSTPICK");
                case "ASCEND_TO_POSTPICK"
                    obj.enter("MOVE_TO_PREPLACE");
                case "MOVE_TO_PREPLACE"
                    obj.log('RRT', 'OFF: reached PRE-PLACE of box %d (%s).', obj.Boxes(k).id, obj.tcpText());
                    obj.enter("DESCEND_TO_PLACE");
                case "DESCEND_TO_PLACE"
                    obj.enter("RELEASE");
                case "RELEASE"
                    obj.enter("ASCEND_TO_POSTPLACE");
                case "ASCEND_TO_POSTPLACE"
                    if k < numel(obj.Boxes)
                        obj.BoxIndex = k + 1;
                        obj.enter("MOVE_TO_PREPICK");
                    else
                        obj.enter("MOVE_TO_HOME");
                    end
                case "MOVE_TO_HOME"
                    obj.log('RRT', 'OFF: robot is at HOME.');
                    obj.enter("DONE");
            end
        end

        function enter(obj, newState)
        %ENTER Switch to newState and prepare its motion.
            obj.State = newState;
            k = obj.BoxIndex;
            b = obj.Boxes(k);
            s = obj.Settings;
            prePick  = b.start + [0, 0, b.h + s.approachHeight];
            pick     = b.start + [0, 0, b.h];
            prePlace = b.goal  + [0, 0, b.h + s.approachHeight];
            place    = b.goal  + [0, 0, b.h];

            try
                switch newState
                    case "MOVE_TO_PREPICK"
                        obj.logState('box %d: RRT to 5 cm above the box on the conveyor', b.id);
                        obj.planRrt(obj.ikOrFail(prePick), []);

                    case "DESCEND_TO_PICK"
                        obj.logState('box %d: down to the box top, z %.3f -> %.3f m (box height %.2f m)', ...
                                     b.id, prePick(3), pick(3), b.h);
                        obj.planVertical(prePick, pick);

                    case "GRASP"
                        obj.logState('box %d: suction ON, box attached', b.id);
                        obj.HeldBox = k;
                        obj.startWait();

                    case "ASCEND_TO_POSTPICK"
                        obj.logState('box %d: up 5 cm with the box, z %.3f -> %.3f m', b.id, pick(3), prePick(3));
                        obj.planVertical(pick, prePick);

                    case "MOVE_TO_PREPLACE"
                        obj.logState('box %d: RRT to 5 cm above the goal [%.3f %.3f %.3f]', b.id, b.goal);
                        obj.planRrt(obj.ikOrFail(prePlace), b);

                    case "DESCEND_TO_PLACE"
                        obj.logState('box %d: down until the box rests on its goal, TCP z %.3f -> %.3f m', ...
                                     b.id, prePlace(3), place(3));
                        obj.planVertical(prePlace, place);

                    case "RELEASE"
                        obj.HeldBox = 0;
                        obj.setBox(k, b.goal);      % exactly at the JSON goal
                        obj.logState('box %d: suction OFF, box placed at [%.3f %.3f %.3f]', b.id, b.goal);
                        if k < numel(obj.Boxes)
                            obj.spawn(k + 1);
                        end
                        obj.startWait();

                    case "ASCEND_TO_POSTPLACE"
                        obj.logState('box %d: up 5 cm, z %.3f -> %.3f m', b.id, place(3), prePlace(3));
                        obj.planVertical(place, prePlace);

                    case "MOVE_TO_HOME"
                        obj.logState('all boxes placed: RRT back to HOME');
                        obj.planRrt(obj.Cfg.homeConfig, []);

                    case "DONE"
                        obj.logState('palletizing finished: %d boxes placed in %.1f s.', numel(obj.Boxes), obj.Time);
                end
            catch err
                obj.State = "FAILED";
                obj.log('ERROR', '%s failed: %s', newState, err.message);
                obj.logState('robot stopped.');
            end
        end

        % ---------------- motions ----------------
        function planRrt(obj, qGoal, heldBox)
            k = obj.BoxIndex;
            ignore = [];
            if ~isempty(heldBox)
                ignore = k;
            end
            obstacles = getCollisionObjects(obj.Boxes, obj.BoxPositions, ignore, rrtConfig().obstacleMargin);
            obj.log('RRT', 'ON: planning ...');
            [waypoints, info] = planJointPath(obj.Tree, obj.Cfg, obj.Q, qGoal, obstacles, heldBox);
            obj.setTrajectory(timeTrajectory(waypoints));
            obj.log('RRT', 'path found in %.2f s (%d waypoints), motion %.1f s.', ...
                    info.planningTime, info.numWaypoints, obj.trajectoryDuration());
        end

        function planVertical(obj, fromTcp, toTcp)
            s = obj.Settings;
            obj.setTrajectory(verticalTrajectory(obj.Tree, obj.Cfg, obj.Q, fromTcp, toTcp, ...
                                                 s.verticalSpeed, s.sampleTime));
        end

        function setTrajectory(obj, qTraj)
            obj.Trajectory = qTraj;
            obj.TrajIndex = 1;
        end

        function startWait(obj)
            obj.WaitSteps = round(obj.Settings.suctionTime / obj.Settings.sampleTime);
        end

        function q = ikOrFail(obj, tcp)
            q = solveTcpIK(obj.Tree, obj.Cfg, tcp, obj.Q);
            if isempty(q)
                error('TCP position [%.3f %.3f %.3f] is not reachable.', tcp);
            end
        end

        % ---------------- boxes ----------------
        function spawn(obj, k)
            b = obj.Boxes(k);
            obj.Env.spawnBox(k);
            obj.BoxPositions(k, :) = b.start;
            obj.log('BOX', 'box %d appeared on the conveyor at [%.3f %.3f %.3f].', b.id, b.start);
        end

        function setBox(obj, k, bottomCentre)
            obj.BoxPositions(k, :) = bottomCentre;
            obj.Env.moveBox(k, bottomCentre);
        end

        % ---------------- helpers ----------------
        function p = tcpPosition(obj)
            T = getTransform(obj.Tree, obj.Q, obj.Cfg.tcpBodyName);
            p = T(1:3, 4)';
        end

        function txt = tcpText(obj)
            txt = sprintf('TCP [%.3f %.3f %.3f]', obj.tcpPosition());
        end

        function d = trajectoryDuration(obj)
            d = (size(obj.Trajectory, 1) - 1) * obj.Settings.sampleTime;
        end

        function logState(obj, varargin)
            logEvent(obj.Time, 'STATE', '%-20s %s', obj.State, sprintf(varargin{:}));
        end

        function log(obj, tag, varargin)
            logEvent(obj.Time, tag, varargin{:});
        end
    end
end
