function cleanupObj = useLatexInterpreters()
    % LaTeX as the default text interpreter until the returned object is cleared
    props = {'defaultAxesTickLabelInterpreter', 'defaultTextInterpreter', 'defaultLegendInterpreter'};
    previous = cellfun(@(p) get(groot, p), props, 'UniformOutput', false);
    cellfun(@(p) set(groot, p, 'latex'), props);
    cleanupObj = onCleanup(@() cellfun(@(p, v) set(groot, p, v), props, previous));
end
