function vc = velocityControlSetup()
    % gains of the rig's per-motor velocity-mode loop, read by Hexapod_PD:
    %   command = Kv*qdRef + Kp*(qRef - q) - Kd*wf*s/(s + wf)*q,   drive speed = a/(s + b)*command (robot.drive)
    % Kp, Kd, Kv are what runs on the rig. The pole placement they started from, kept for reference:
    %   bw = 21 Hz, zeta = 1, wn = 2*pi*bw/sqrt(1 - 2*zeta^2 + sqrt(2 - 4*zeta^2 + 4*zeta^4)),
    %   p3 = b + wf - 2*zeta*wn, Kp = p3*wn^2/(a*wf), Kd = (wn^2 + 2*zeta*wn*p3 - b*wf - a*Kp)/(a*wf), Kv = Kd + b/a
    vc.Kp = 101;
    vc.Kd = 1.1;
    vc.Kv = 0.75;
    vc.wf = 2*pi*100;   % derivative filter, rad/s
end
