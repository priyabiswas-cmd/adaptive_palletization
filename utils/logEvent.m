function logEvent(simTime, tag, varargin)
%LOGEVENT Print one line to the terminal: simulation time, tag, message.
%   logEvent(3.42, 'STATE', 'DESCEND_TO_PICK (box %d)', 1)
%   -> [   3.42 s] [STATE]  DESCEND_TO_PICK (box 1)

    fprintf('[%7.2f s] %-8s %s\n', simTime, ['[' tag ']'], sprintf(varargin{:}));
end
