function boxes = loadTaskPlan(jsonPath)
%LOADTASKPLAN Read the palletizing task plan (JSON) and check it.
%   boxes = loadTaskPlan(jsonPath) returns a struct array with one element
%   per box, in pick order. Each element has the fields:
%       id, l, b, h, weight, start [x y z], goal [x y z]  (weight in kg)
%   Positions are in the world frame (meters). [x y] is the centre of the
%   box bottom face and z is the height of the bottom face.
%   Boxes appear one after another at their start, so starts may be equal.

    if nargin < 1
        jsonPath = fullfile(fileparts(mfilename('fullpath')), '..', 'config', 'boxes.json');
    end
    if ~isfile(jsonPath)
        error('loadTaskPlan:fileNotFound', 'Task plan not found: %s', jsonPath);
    end

    raw = jsondecode(fileread(jsonPath));
    if ~isfield(raw, 'boxes') || isempty(raw.boxes)
        error('loadTaskPlan:noBoxes', 'Task plan has no "boxes" list.');
    end

    rawBoxes = raw.boxes;
    if iscell(rawBoxes)   % jsondecode returns a cell array if boxes differ in fields
        rawBoxes = [rawBoxes{:}];
    end

    numBoxes = numel(rawBoxes);
    boxes = struct('id', cell(1, numBoxes), 'l', [], 'b', [], 'h', [], ...
                   'weight', [], 'start', [], 'goal', []);

    for k = 1:numBoxes
        rb = rawBoxes(k);
        checkFields(rb, {'id', 'l', 'b', 'h', 'weight', 'start', 'goal'}, sprintf('box #%d', k));
        checkFields(rb.start, {'x', 'y', 'z'}, sprintf('box %d "start"', rb.id));
        checkFields(rb.goal,  {'x', 'y', 'z'}, sprintf('box %d "goal"', rb.id));

        boxes(k).id    = rb.id;
        boxes(k).l     = rb.l;
        boxes(k).b     = rb.b;
        boxes(k).h     = rb.h;
        boxes(k).weight = rb.weight;
        boxes(k).start = [rb.start.x, rb.start.y, rb.start.z];
        boxes(k).goal  = [rb.goal.x,  rb.goal.y,  rb.goal.z];

        if any([rb.l, rb.b, rb.h, rb.weight] <= 0)
            error('loadTaskPlan:badSize', 'Box %d: l, b, h and weight must be positive.', rb.id);
        end
    end

    ids = [boxes.id];
    if numel(unique(ids)) ~= numel(ids)
        error('loadTaskPlan:duplicateId', 'Box ids must be unique.');
    end

    % Boxes appear one after another, so several may share a start position.
    % Their goals on the pallet must not overlap.
    checkNoOverlap(boxes, 'goal');
end

function checkFields(s, names, where)
    missing = names(~isfield(s, names));
    if ~isempty(missing)
        error('loadTaskPlan:missingField', '%s is missing: %s', where, strjoin(missing, ', '));
    end
end

function checkNoOverlap(boxes, which)
%CHECKNOOVERLAP Error if two boxes would occupy the same space at their goals.
    for i = 1:numel(boxes)
        for j = i+1:numel(boxes)
            [minI, maxI] = boxBounds(boxes(i), boxes(i).(which));
            [minJ, maxJ] = boxBounds(boxes(j), boxes(j).(which));
            if all(minI < maxJ & minJ < maxI)
                error('loadTaskPlan:overlap', 'Boxes %d and %d overlap at their %s positions.', ...
                      boxes(i).id, boxes(j).id, which);
            end
        end
    end
end

function [lo, hi] = boxBounds(box, pos)
    half = [box.l, box.b] / 2;
    lo = [pos(1:2) - half, pos(3)];
    hi = [pos(1:2) + half, pos(3) + box.h];
end
