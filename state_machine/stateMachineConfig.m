function c = stateMachineConfig()
%STATEMACHINECONFIG Settings of the pick-and-place state machine.

    c.approachHeight = 0.05;   % m above the box top: pre-pick, post-pick, pre-place, post-place
    c.verticalSpeed  = 0.05;   % m/s, peak speed of the straight up/down moves
    c.suctionTime    = 0.5;    % s, wait at GRASP and RELEASE (suction on / off)
    c.sampleTime     = 0.02;   % s, one step() call = one controller step
end
