function jointAngles = debugStep()
    n = 1;
    dt = 1e-3;

    % =============================================================
    % User settings
    % =============================================================

    n_joints = 6;

    % Unit amplitude for every joint
    A = 1.0;

    % Every duration is 1 second
    rise_time         = 1.0;
    hold_time         = 1.0;
    fall_time         = 1.0;
    zero_between_sign = 1.0;
    zero_between_axis = 1.0;

    % Initial zero before the sequence begins
    initial_zero_time = 1.0;


    % =============================================================
    % Timing
    %
    % For each joint:
    %
    %   0
    %   -> rise to +1
    %   -> hold +1
    %   -> fall to 0
    %   -> hold 0
    %   -> rise to -1
    %   -> hold -1
    %   -> fall to 0
    %   -> hold 0
    %   -> next joint
    %
    % =============================================================

    bump_time = ...
        rise_time + ...
        hold_time + ...
        fall_time;

    axis_time = ...
        bump_time + ...
        zero_between_sign + ...
        bump_time + ...
        zero_between_axis;

    sequence_time = ...
        initial_zero_time + ...
        n_joints * axis_time;

    tf = n * sequence_time;


    % =============================================================
    % Allocate entire arrays up front
    % =============================================================

    time = (0:dt:tf)';

    N = length(time);

    q = zeros(N, n_joints);


    % =============================================================
    % Generate debug sequence
    % =============================================================

    for repetition = 1:n

        % Beginning of this repetition
        sequence_start = ...
            (repetition - 1) * sequence_time;

        % Initial zero hold
        current_time = ...
            sequence_start + initial_zero_time;


        % ---------------------------------------------------------
        % Joint-by-joint excitation
        % ---------------------------------------------------------

        for joint = 1:n_joints

            % =====================================================
            % Positive excitation
            %
            %      0 -> +1 -> hold -> 0
            % =====================================================

            a = current_time;
            b = a + bump_time;

            q(:,joint) = q(:,joint) + ...
                A * bumpwin( ...
                    time, ...
                    a, ...
                    rise_time, ...
                    fall_time, ...
                    b);

            current_time = b;


            % =====================================================
            % Zero hold
            % =====================================================

            current_time = ...
                current_time + zero_between_sign;


            % =====================================================
            % Negative excitation
            %
            %      0 -> -1 -> hold -> 0
            % =====================================================

            a = current_time;
            b = a + bump_time;

            q(:,joint) = q(:,joint) - ...
                A * bumpwin( ...
                    time, ...
                    a, ...
                    rise_time, ...
                    fall_time, ...
                    b);

            current_time = b;


            % =====================================================
            % Zero hold before next joint
            % =====================================================

            current_time = ...
                current_time + zero_between_axis;

        end

    end


    % =============================================================
    % Output
    %
    % jointangles =
    %
    %   [time, q1, q2, q3, q4, q5, q6]
    %
    % =============================================================

    jointAngles = [time, q];

end
