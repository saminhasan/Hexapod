function [cv, qName, qdName, p] = torqueServo(s, k, extraName, extraTable)
    % switches servo subsystem s (motor k) from angle driven to torque driven and leaves three reference
    % lookups: qRef (the old angle lookup), qdRef (ref.qd) and extraName (extraTable), all the same cubic
    % spline on ref.t. Returns the converter name, the sensed angle/rate block names and qRef's position
    r = [s '/Revolute Joint1'];
    cv = find_system(s, 'SearchDepth', 1, 'Regexp', 'on', 'Name', '^Simulink-PS Converter'); % ...Converter or ...Converter1
    cv = get_param(cv{1}, 'Name');
    pc = get_param([s '/Mux'], 'PortConnectivity');
    qName = get_param(pc(1).SrcBlock, 'Name');
    qdName = get_param(pc(2).SrcBlock, 'Name');

    % torque in, motion out; torque sensing stays on, so motorData slot 4 is the applied torque
    set_param(r, 'TorqueActuationMode', 'InputTorque', 'MotionActuationMode', 'ComputedMotion');
    set_param([s '/' cv], 'FilteringAndDerivatives', 'zero', 'Unit', 'N*m');
    ph = get_param([s '/' cv], 'PortHandles'); pr = get_param(r, 'PortHandles');
    if get_param(pr.LConn(2), 'Line') < 0 % the mode switch drops the physical line to the actuation port
        add_line(s, ph.RConn(1), pr.LConn(2), 'autorouting', 'on');
    end

    delete_line(s, '1-D Lookup Table/1', [cv '/1']);
    set_param([s '/1-D Lookup Table'], 'Name', 'qRef');
    p = get_param([s '/qRef'], 'Position');
    add_block([s '/qRef'], [s '/qdRef'], 'Table', sprintf('ref.qd(:,%d)', k), 'Position', p + [0 60 0 60]);
    add_block([s '/qRef'], [s '/' extraName], 'Table', extraTable, 'Position', p + [0 120 0 120]);
    add_line(s, 'time/1', 'qdRef/1', 'autorouting', 'on');
    add_line(s, 'time/1', [extraName '/1'], 'autorouting', 'on');
end
