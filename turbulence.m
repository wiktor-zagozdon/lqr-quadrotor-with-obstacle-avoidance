function next_turb = turbulence(prev_turb)
    % Initialize to zero if empty on the first call
    if nargin < 1 || isempty(prev_turb)
        prev_turb = [0.0; 0.0; 0.0];
    end

    % --- YOUR TWO TUNING KNOBS ---
    max_amplitude  = 4.0;   % Knob 1: Capped peak wind speed (m/s)
    rate_of_change = 0.03;  % Knob 2: How fast the wind shifts (0.01 to 0.2)

    % 1. Get a completely fresh random target vector (-1 to 1)
    raw_random = 2 * rand(3, 1) - 1;

    % 2. Simple Low-Pass Filter with a booster multiplier (3.0)
    % The 3.0 booster forces the random walk to aggressively reach the limits.
    next_gust = (1 - rate_of_change) * prev_turb + (rate_of_change * max_amplitude * 3.0 * raw_random);

    % 3. Hard clamp so the boosted waves perfectly level off at your max amplitude
    next_turb = max(min(next_gust, max_amplitude), -max_amplitude);
end