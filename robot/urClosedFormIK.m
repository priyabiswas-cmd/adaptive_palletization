function sols = urClosedFormIK(dh, T06)
%URCLOSEDFORMIK All closed-form IK solutions (up to 8) for a Universal Robots arm.
%   sols = urClosedFormIK(dh, T06)
%     dh   : struct with fields a, d, alpha (standard DH, 1x6 each)
%     T06  : 4x4 flange pose expressed in the DH base frame
%     sols : k x 6 joint angles (not yet wrapped or limit-checked)
%   Branches: shoulder left/right, wrist up/down, elbow up/down.
%   UR arms have no spherical wrist, so the toolbox's
%   analyticalInverseKinematics cannot solve them; this is the standard
%   geometric solution for the UR family.

    a = dh.a; d = dh.d; alpha = dh.alpha;
    sols = zeros(0, 6);

    % Joint 1, from the wrist-2 centre (frame 5 origin)
    P05 = T06 * [0; 0; -d(6); 1];
    r = hypot(P05(1), P05(2));
    if r < abs(d(4))
        return;   % target too close to the base axis
    end
    phi1 = atan2(P05(2), P05(1));
    phi2 = acos(d(4) / r);
    P06 = T06(1:3, 4);
    T60 = invTransform(T06);

    for th1 = phi1 + [phi2, -phi2] + pi/2
        s1 = sin(th1); c1 = cos(th1);

        % Joint 5
        val = (P06(1) * s1 - P06(2) * c1 - d(4)) / d(6);
        if abs(val) > 1 + 1e-9
            continue;
        end
        val = max(-1, min(1, val));

        for th5 = [acos(val), -acos(val)]
            s5 = sin(th5);

            % Joint 6 (free when the wrist is singular; choose 0)
            if abs(s5) < 1e-9
                th6 = 0;
            else
                th6 = atan2((-T60(2, 1) * s1 + T60(2, 2) * c1) / s5, ...
                            ( T60(1, 1) * s1 - T60(1, 2) * c1) / s5);
            end

            % Joints 2, 3, 4: planar 3-link problem
            T01 = dhTransform(a(1), alpha(1), d(1), th1);
            T45 = dhTransform(a(5), alpha(5), d(5), th5);
            T56 = dhTransform(a(6), alpha(6), d(6), th6);
            T14 = invTransform(T01) * T06 * invTransform(T45 * T56);
            P13 = T14 * [0; -d(4); 0; 1];
            L = norm(P13(1:3));
            cos3 = (L^2 - a(2)^2 - a(3)^2) / (2 * a(2) * a(3));
            if abs(cos3) > 1 + 1e-9
                continue;   % out of reach
            end
            cos3 = max(-1, min(1, cos3));

            for th3 = [acos(cos3), -acos(cos3)]
                th2 = -atan2(P13(2), -P13(1)) + asin(a(3) * sin(th3) / L);
                T12 = dhTransform(a(2), alpha(2), d(2), th2);
                T23 = dhTransform(a(3), alpha(3), d(3), th3);
                T34 = invTransform(T12 * T23) * T14;
                th4 = atan2(T34(2, 1), T34(1, 1));
                sols(end+1, :) = [th1, th2, th3, th4, th5, th6]; %#ok<AGROW>
            end
        end
    end
end

function Ti = invTransform(T)
    R = T(1:3, 1:3);
    Ti = [R', -R' * T(1:3, 4); 0 0 0 1];
end
