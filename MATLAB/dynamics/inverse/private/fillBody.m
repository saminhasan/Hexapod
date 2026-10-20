function p = fillBody(p)
    % checks a body model (bodyModels) and fills it out: COMs and gravity as columns, limb values 6x1,
    % viscousFriction 0 and gearRatio 9 when missing
    names = {'platformMass', 'platformCOM', 'platformInertia', 'torsoMass', 'torsoCOM', 'torsoInertia', ...
        'rodMass', 'armMass', 'armCOM', 'armInertia', 'reflectedInertia', 'gravity'};
    if ~isstruct(p) || ~isscalar(p) || ~all(isfield(p, names))
        error('inverseDynamics:Body', 'body must be a scalar struct with %s', strjoin(names, ', '));
    end
    if ~isfield(p, 'viscousFriction'), p.viscousFriction = 0; end
    if ~isfield(p, 'gearRatio'), p.gearRatio = 9; end
    validateattributes(p.platformMass, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
    validateattributes(p.torsoMass, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
    validateattributes(p.gearRatio, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
    for f = {'platformCOM', 'torsoCOM', 'gravity'}
        validateattributes(p.(f{1}), {'numeric'}, {'real', 'finite', 'vector', 'numel', 3});
        p.(f{1}) = p.(f{1})(:);
    end
    for f = {'platformInertia', 'torsoInertia'}
        I = p.(f{1});
        validateattributes(I, {'numeric'}, {'real', 'finite', 'size', [3 3]});
        if norm(I - I.', 'fro') > 1e-10*max(1, norm(I, 'fro')) || min(eig((I + I.')/2)) < -1e-12
            error('inverseDynamics:Inertia', 'body.%s must be symmetric positive semidefinite', f{1});
        end
    end
    for f = {'rodMass', 'armMass', 'armCOM', 'armInertia', 'reflectedInertia', 'viscousFriction'}
        v = p.(f{1});
        validateattributes(v, {'numeric'}, {'real', 'finite', 'vector', 'nonnegative'});
        if isscalar(v)
            p.(f{1}) = repmat(v, 6, 1);
        elseif numel(v) == 6
            p.(f{1}) = v(:);
        else
            error('inverseDynamics:Body', 'body.%s must be scalar or six elements', f{1});
        end
    end
end
