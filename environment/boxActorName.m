function name = boxActorName(id)
%BOXACTORNAME Name of the 3D actor for the box with this JSON id (e.g. 'box1').
    name = sprintf('box%d', id);
end
