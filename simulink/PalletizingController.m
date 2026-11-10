classdef PalletizingController < matlab.System
%PALLETIZINGCONTROLLER "Task Controller" block of palletizing.slx.
%   Runs every simulation step (0.02 s) and outputs the robot joint angles
%   (6x1) to the Simulation 3D Robot block. It only connects Simulink to the
%   project code:
%     - the task logic is state_machine/PalletizingStateMachine
%     - boxes are shown and moved in the 3D world with environment/spawnBox
%       and environment/setBoxPosition

    properties (Access = private)
        StateMachine    % PalletizingStateMachine
        Boxes           % task plan from config/boxes.json
        World           % sim3d world of the running model
    end

    methods (Access = protected)
        function setupImpl(obj)
            [tree, cfg] = setupRobot();
            obj.Boxes = loadTaskPlan();
            env.spawnBox = @(k) spawnBox(obj.world(), obj.Boxes(k));
            env.moveBox  = @(k, p) setBoxPosition(obj.world(), obj.Boxes(k), p);
            obj.StateMachine = PalletizingStateMachine(tree, cfg, obj.Boxes, env);
            fprintf('[Controller] Ready: %d boxes in config/boxes.json, robot at HOME.\n', numel(obj.Boxes));
        end

        function q = stepImpl(obj)
            q = obj.StateMachine.step();
            q = q(:);
        end

        % ---- Simulink interface: one 6x1 double output, 0.02 s step ----
        function num = getNumInputsImpl(~),  num = 0; end
        function num = getNumOutputsImpl(~), num = 1; end
        function sz = getOutputSizeImpl(~),  sz = [6 1]; end
        function dt = getOutputDataTypeImpl(~), dt = 'double'; end
        function c = isOutputComplexImpl(~), c = false; end
        function f = isOutputFixedSizeImpl(~), f = true; end
        function st = getSampleTimeImpl(obj)
            st = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', 0.02);
        end
        function name = getOutputNamesImpl(~), name = 'q'; end
    end

    methods (Access = private)
        function w = world(obj)
        %WORLD The 3D world of the running model (looked up once).
            if isempty(obj.World)
                obj.World = sim3d.World.getWorld(bdroot);
            end
            w = obj.World;
        end
    end
end
