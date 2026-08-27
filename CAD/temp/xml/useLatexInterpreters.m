function cleanupObj = useLatexInterpreters()
%USELATEXINTERPRETERS  Temporarily make LaTeX the default text interpreter.
%
%   cleanupObj = useLatexInterpreters();
%
%   Hold the returned object for as long as the figures are being built:
%
%       cleanupObj = useLatexInterpreters(); %#ok<NASGU>
%
%   The previous groot defaults are restored when cleanupObj goes out of
%   scope, so a plotting function no longer leaks its interpreter choice
%   into every figure drawn later in the session.
%
%   Objects created while the defaults were active keep LaTeX: the
%   interpreter is baked into each object at creation time, not looked up
%   again at draw time. Restoring groot afterwards leaves them alone.

    props = { ...
        'defaultAxesTickLabelInterpreter', ...
        'defaultTextInterpreter', ...
        'defaultLegendInterpreter'};

    previous = cell(size(props));

    for k = 1:numel(props)
        previous{k} = get(groot, props{k});
        set(groot, props{k}, 'latex');
    end

    cleanupObj = onCleanup(@() restoreInterpreters(props, previous));

end


function restoreInterpreters(props, previous)
    for k = 1:numel(props)
        set(groot, props{k}, previous{k});
    end
end
