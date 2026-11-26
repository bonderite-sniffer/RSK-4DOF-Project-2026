function mdl = build_rs05_model(port)
%BUILD_RS05_MODEL  Create rs05_sim.slx: Simulink host model for 4x RS05 over USB serial.
%
%   build_rs05_model("COM5")
%   then press Run. Change the Constant blocks WHILE it runs to command the motors
%   (double-click -> edit value -> OK). Flip the Manual Switch to stream a sine
%   trajectory to the position target.
%
%   Requires: MATLAB + Simulink R2022a or newer. No STM32 blockset.

    if nargin < 1, port = "COM5"; end
    mdl = 'rs05_sim';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
    new_system(mdl);
    open_system(mdl);

    % real-time-ish host loop: fixed 10 ms step, paced to wall clock
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
        'FixedStep', '0.01', 'StopTime', 'inf', 'EnablePacing', 'on', 'PacingRate', '1');

    c = @(name, val, pos) add_block('simulink/Sources/Constant', [mdl '/' name], ...
        'Value', val, 'Position', pos);

    % ---- command inputs
    c('enable',     '0',            [40  40 110  70]);   % start DISABLED; set to 1 to drive
    c('mode',       '3',            [40 100 110 130]);   % 1 MIT, 2 VEL, 3 POS
    c('mask',       '1',            [40 160 110 190]);   % bit mask: 1 M1, 2 M2, 4 M3, 8 M4, 15 all
    c('MIT_params', '[0 0 0 0.2 0]',[20 220 130 250]);   % [p_deg v Kp Kd tau_ff]
    c('VEL_params', '[10 3 10]',    [20 280 130 310]);   % [rpm A rad/s^2]

    c('pos_target_deg', '0',        [20 340 110 370]);
    add_block('simulink/Sources/Sine Wave', [mdl '/sine_deg'], ...
        'Amplitude', '45', 'Frequency', num2str(2*pi*0.1), 'SampleTime', '0.01', ...
        'Position', [20 400 60 430]);                    % 45 deg, 0.1 Hz
    add_block('simulink/Signal Routing/Manual Switch', [mdl '/target_select'], ...
        'Position', [160 370 190 400]);
    c('POS_limits', '[1 3]',        [150 440 230 470]);  % [rad/s A]
    add_block('simulink/Signal Routing/Mux', [mdl '/pos_mux'], 'Inputs', '2', ...
        'Position', [260 380 265 460]);

    % ---- the RS05 block
    blk = [mdl '/RS05'];
    add_block('simulink/User-Defined Functions/MATLAB System', blk, ...
        'System', 'RS05Block', 'Position', [360 60 520 460]);
    set_param(blk, 'Port', sprintf('''%s''', port));

    % ---- outputs
    names = {'Posisi_deg', 'Kecepatan_rpm', 'Torsi_Nm', 'Temperatur_C', 'Arus_iq_A'};
    for k = 1:5
        y = 60 + (k - 1) * 85;
        add_block('simulink/Sinks/Scope', [mdl '/' names{k}], 'Position', [640 y 680 y + 40]);
        add_line(mdl, sprintf('RS05/%d', k), [names{k} '/1'], 'autorouting', 'on');
    end

    % ---- wiring
    add_line(mdl, 'enable/1',         'RS05/1', 'autorouting', 'on');
    add_line(mdl, 'mode/1',           'RS05/2', 'autorouting', 'on');
    add_line(mdl, 'mask/1',           'RS05/3', 'autorouting', 'on');
    add_line(mdl, 'MIT_params/1',     'RS05/4', 'autorouting', 'on');
    add_line(mdl, 'VEL_params/1',     'RS05/5', 'autorouting', 'on');
    add_line(mdl, 'pos_target_deg/1', 'target_select/1', 'autorouting', 'on');
    add_line(mdl, 'sine_deg/1',       'target_select/2', 'autorouting', 'on');
    add_line(mdl, 'target_select/1',  'pos_mux/1', 'autorouting', 'on');
    add_line(mdl, 'POS_limits/1',     'pos_mux/2', 'autorouting', 'on');
    add_line(mdl, 'pos_mux/1',        'RS05/6', 'autorouting', 'on');

    note = Simulink.Annotation(mdl, sprintf([ ...
        'RS05 x4 via USB serial (no STM32 blockset) - outputs are 4x1 [M1..M4]\n' ...
        'enable 0/1 | mode 1=MIT 2=VEL 3=POS | mask bits M1=1 M2=2 M3=4 M4=8 (15 all)\n' ...
        'MIT [p_deg v Kp Kd tau_ff]  VEL [rpm A rad/s^2]  POS limits [rad/s A]\n' ...
        'Edit constants while running. Stop = E-STOP. Zeroing: use RS05App.']));
    note.Position = [40 500 520 570];

    save_system(mdl);
    fprintf('Model %s.slx created for port %s. Press Run.\n', mdl, port);
end
