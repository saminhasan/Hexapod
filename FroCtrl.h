#ifndef FROCTRL_H
#define FROCTRL_H


class FeedrateGovernor {
public:
    // All parameters and internal state variables are floats and marked as volatile.
    FeedrateGovernor(float initial = 0.0f, float scale_ms_per_unit = 10.0f, float feedrate_max = 100.0f)
        : _fro(initial), _from(initial), _to(initial), _scale(scale_ms_per_unit),
          _frMax(feedrate_max), _elapsed(0.0f), _duration(0.0f), _active(false) {}

    // Set a new target feedrate.
    // If a transition is already in progress, this blends smoothly from the current output.
    void setTarget(float target) {
        
        if (target > _frMax)
            target = _frMax;

        if (target == _fro) {
            return;
        }

        _from   = _fro;  // Start new transition from current output.
        _to     = target;
        _elapsed = 0.0f;
        // Duration based on absolute difference, scaled by _scale.
        _duration = abs(_to - _from) * _scale;
        if (_duration < 1.0f)  // Avoid zero-duration transitions.
            _duration = 1.0f;
        _active = true;
        
    }

    // Call this every tick. dt is in milliseconds (default 1ms).
    void tock(float dt = 1.0f) {
        if (!_active)
            return;

        _elapsed += dt;
        float x = _elapsed / _duration;
        if (x >= 1.0f) {
            _fro = _to;
            _active = false;
        } else {
            // Compute smooth step factor (in range 0..1)
            float factor = smoothStep(x);
            _fro = _from + (_to - _from) * factor;
        }
    }

    // Get the current feedrate.
    float getFeedrate() const {
        return _fro;
    }

    // Get the current target feedrate.
    float getTarget() const {
        return _to;
    }

private:
    volatile float _fro;       // Current feedrate output.
    volatile float _from;      // Feedrate at the start of the current transition.
    volatile float _to;        // Target feedrate.
    volatile float _scale;     // Time scaling factor (ms per unit difference).
    volatile float _frMax;     // Maximum allowed feedrate.
    volatile float _elapsed;   // Elapsed time (ms) since transition start.
    volatile float _duration;  // Total duration (ms) of the transition.
    volatile bool  _active;    // Is a transition in progress?

    // Smooth step function that is infinitely differentiable.
    // Defined for x in (0, 1) with h(0)=0 and h(1)=1.
    float smoothStep(float x) {
        if (x <= 0.0f)
            return 0.0f;
        if (x >= 1.0f)
            return 1.0f;
        float a = exp(-1.0f / x);
        float b = exp(-1.0f / (1.0f - x));
        return a / (a + b);
    }
};

#endif // FROCTRL_H
