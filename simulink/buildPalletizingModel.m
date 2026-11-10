function buildPalletizingModel()
%BUILDPALLETIZINGMODEL Create palletizing.slx (the simulator shell) in the project folder.
%   Same simulator and camera as the MathWorks example
%   "Palletize Boxes Using Cobot with Simulink 3D Animation":
%     - Simulation 3D Scene Configuration : Unreal "Empty scene", example camera
%     - Simulation 3D Actor "Work Cell"   : buildWorkCell(World, Actor)
%     - Simulation 3D Robot "Robot"       : UR10e + EPick from setupRobot()
%     - MATLAB System "Task Controller"   : PalletizingController -> joint angles
%   Re-run this function whenever you want to rebuild the model from scratch.

    root = fileparts(fileparts(mfilename('fullpath')));
    model = 'palletizing';
    file = fullfile(root, [model '.slx']);
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    if isfile(file)
        delete(file);
    end

    env = environmentConfig();
    new_system(model);

    % Model settings (as in the example): fixed 0.01 s base step, run until
    % stopped, real-time pacing. Scene and controller run every 0.02 s.
    set_param(model, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.01', ...
        'StopTime', 'inf', 'EnablePacing', 'on', 'PacingRate', '1');

    % Load paths and the robot model whenever the model is opened or run
    initCode = sprintf(['run(fullfile(fileparts(get_param(bdroot, ''FileName'')), ''setupPaths.m''));\n' ...
                        '[rbt, robotCfg] = setupRobot();\n' ...
                        'robotHome = robotCfg.homeConfig;\n']);
    set_param(model, 'PreLoadFcn', initCode, 'InitFcn', initCode);

    % 1) Scene configuration: same scene and camera as the example
    scene = [model '/Simulation 3D Scene Configuration'];
    add_block('sim3dlib/Simulation 3D Scene Configuration', scene, 'Position', [40 40 220 120]);
    set_param(scene, 'SceneDesc', 'Empty scene', ...
        'FreeCameraDefaultInitialTranslation', mat2str(env.camera.translation), ...
        'FreeCameraDefaultInitialRotation', mat2str(env.camera.rotation), ...
        'HorizontalFOV', num2str(env.camera.fieldOfView), ...
        'HorizontalResolution', num2str(env.camera.resolution(1)), ...
        'VerticalResolution', num2str(env.camera.resolution(2)), ...
        'Ts', '0.02');

    % 2) Work cell: floor, platform, static conveyor, pallet, boxes from the JSON
    workCell = [model '/Work Cell'];
    add_block('sim3dlib/Simulation 3D Actor', workCell, 'Position', [40 180 220 240]);
    set_param(workCell, 'ActorName', 'WorkCell', 'ParentName', 'Scene Origin', ...
        'Operation', 'Create at setup', 'InitScriptText', 'buildWorkCell(World, Actor);');
    add_block('simulink/Sinks/Terminator', [model '/Work Cell Out'], 'Position', [270 200 290 220]);
    add_line(model, 'Work Cell/1', 'Work Cell Out/1');

    % 3) Task controller (MATLAB code) -> 4) robot in the 3D world
    ctrl = [model '/Task Controller'];
    add_block('simulink/User-Defined Functions/MATLAB System', ctrl, 'Position', [40 300 220 360]);
    set_param(ctrl, 'System', 'PalletizingController', 'SimulateUsing', 'Interpreted execution');

    % The Robot block checks its parameters immediately, so the variables
    % it refers to must already exist in the base workspace
    [rbt, robotCfg] = setupRobot();
    assignin('base', 'rbt', rbt);
    assignin('base', 'robotCfg', robotCfg);
    assignin('base', 'robotHome', robotCfg.homeConfig);

    robot = [model '/Robot'];
    add_block('robotsim3dlib/Simulation 3D Robot', robot, 'Position', [320 300 500 360]);
    set_param(robot, 'RigidBodyTree', 'rbt', 'ActorName', 'Robot', ...
        'initialConfig', 'robotHome', 'EndEffector', '4cup_assembly');
    add_line(model, 'Task Controller/1', 'Robot/1');

    save_system(model, file);
    fprintf('Created %s\n', file);
end
