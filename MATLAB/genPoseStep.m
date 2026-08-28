function trajectory = genPoseStep(n)

    dt = 1e-3;

    % =============================================================
    % User settings
    % =============================================================

    % Translational amplitudes [m]
    Ax = 0.01;
    Ay = 0.01;
    Az = 0.04;

    % Rotational amplitudes [rad]
    ARx = deg2rad(10.0);
    ARy = deg2rad(10.0);
    ARz = deg2rad(15.0);


    % -------------------------------------------------------------
    % Timing [s]
    %
    % Sequence for EACH axis:
    %
    %   zero
    %       -> rise to +A
    %       -> hold +A
    %       -> fall to 0
    %       -> zero hold
    %       -> rise to -A
    %       -> hold -A
    %       -> fall to 0
    %       -> zero hold
    %       -> next axis
    %
    % -------------------------------------------------------------

    initial_zero_time = 1.0;   % Zero before each complete 6-axis sequence

    rise_time         = 1.0;  % 0 -> 100 %
    hold_time         = 1.0;  % Hold at +/- amplitude
    fall_time         = 1.0;  % 100 -> 0 %

    zero_between_sign = 1.0;  % Zero between + and - motion
    zero_between_axis = 1.0;  % Zero before next axis


    % =============================================================
    % Axis information
    % =============================================================

    % Columns correspond to:
    %
    %   1 = x
    %   2 = y
    %   3 = z
    %   4 = Rx
    %   5 = Ry
    %   6 = Rz

    amplitudes = [Ax, Ay, Az, ARx, ARy, ARz];

    n_axes = length(amplitudes);


    % =============================================================
    % Compute complete sequence duration
    % =============================================================

    % Duration of one + or - bump:
    %
    %       rise -> hold -> fall
    %
    bump_time = rise_time + hold_time + fall_time;


    % Duration allocated to one axis:
    %
    %       + bump
    %       zero
    %       - bump
    %       zero
    %
    axis_time = ...
        bump_time          + ...
        zero_between_sign  + ...
        bump_time          + ...
        zero_between_axis;


    % One complete x,y,z,Rx,Ry,Rz sequence
    sequence_time = ...
        initial_zero_time + ...
        n_axes * axis_time;


    % Repeat complete sequence n times
    tf = n * sequence_time;


    % =============================================================
    % Allocate everything UP FRONT
    % =============================================================

    time = (0:dt:tf)';

    N = length(time);

    % Entire pose trajectory allocated once
    pose = zeros(N, 6);


    % =============================================================
    % Generate sequences
    % =============================================================

    for repetition = 1:n

        % Start of this full 6-axis sequence
        sequence_start = ...
            (repetition - 1) * sequence_time;


        % First motion begins after initial zero period
        current_time = ...
            sequence_start + initial_zero_time;


        % ---------------------------------------------------------
        % Go through x, y, z, Rx, Ry, Rz
        % ---------------------------------------------------------

        for axis = 1:n_axes

            A = amplitudes(axis);


            % =====================================================
            % Positive motion
            %
            %       0 -> +A -> hold -> 0
            % =====================================================

            a = current_time;

            b = a + bump_time;

            pose(:,axis) = pose(:,axis) + ...
                A * bumpwin( ...
                    time, ...
                    a, ...
                    rise_time, ...
                    fall_time, ...
                    b);


            % Move to end of positive bump
            current_time = b;


            % =====================================================
            % Zero hold between + and -
            % =====================================================

            current_time = ...
                current_time + zero_between_sign;


            % =====================================================
            % Negative motion
            %
            %       0 -> -A -> hold -> 0
            % =====================================================

            a = current_time;

            b = a + bump_time;

            pose(:,axis) = pose(:,axis) - ...
                A * bumpwin( ...
                    time, ...
                    a, ...
                    rise_time, ...
                    fall_time, ...
                    b);


            % Move to end of negative bump
            current_time = b;


            % =====================================================
            % Zero before next axis
            % =====================================================

            current_time = ...
                current_time + zero_between_axis;

        end

    end


    % =============================================================
    % Output
    %
    % trajectory =
    %
    %   [ time   x   y   z   Rx   Ry   Rz ]
    %
    % =============================================================

    trajectory = [time, pose];

end
