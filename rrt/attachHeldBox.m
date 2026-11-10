function planTree = attachHeldBox(tree, cfg, box, margin)
%ATTACHHELDBOX Copy of the robot tree with the carried box attached under the suction cups.
%   The box hangs from the TCP: its top face touches the cup faces. It is
%   attached to the '4cup_assembly' body so the planner does not report the
%   box touching its own cups as a self-collision ("Parent" pairs are skipped).

    planTree = copy(tree);
    if isempty(box)
        return;
    end

    % Box centre in the TCP frame: TCP z points down into the box
    Ttcp_box = trvec2tform([0, 0, box.h/2]);
    % TCP pose in the 4cup_assembly frame (fixed offset)
    T4cup_tcp = getTransform(tree, cfg.homeConfig, cfg.tcpBodyName, '4cup_assembly');

    body = rigidBody('heldBox');
    joint = rigidBodyJoint('heldBoxJoint', 'fixed');
    setFixedTransform(joint, T4cup_tcp * Ttcp_box);
    body.Joint = joint;
    shape = collisionBox(box.l + 2*margin, box.b + 2*margin, box.h);
    addCollision(body, shape);
    addBody(planTree, body, '4cup_assembly');
end
