%% example_script.m - scripted test with RS05Link (protocol v2, 4 motors)
% Position test 90 / -90 / 45 / -45 deg on M1 and M2, logged to CSV.

port = "COM8";                         % <-- USB-C port of the board
L = RS05Link(port);
cleanup = onCleanup(@() delete(L));    % E-STOP + close on exit/error

info = L.ping();
fprintf('fw %s, %d Hz, %d motors\n', info.fw, info.ctrlHz, info.nMotors);
L.setCfg('TELEM_DIV', 5);              % 100 Hz telemetry
L.setCfg('TSCALE', 5.5);               % RS05 torque range +-5.5 Nm
L.setCfg('POS_MIN', deg2rad(-110));
L.setCfg('POS_MAX', deg2rad(110));
ids = [1 2 1 2];                        % CAN IDs of M1..M4
for k = 1:4, L.setMotorId(k, ids(k)); end

r = L.scan(3);
for b = 1:numel(r), fprintf('CAN%d ids: %s\n', r{b}.bus, mat2str(r{b}.ids)); end

M = RS05Link.mask([1 2]);
pause(0.3);
assert(all(L.Latest.online(1:2)), 'M1/M2 offline - check wiring / IDs');

L.startRecording();
for deg = [90 -90 45 -45]
    L.pos(M, deg2rad(deg), 1.0, 3.0);
    t0 = tic;
    while any(abs(L.Latest.pos(1:2) - deg2rad(deg)) > deg2rad(1)) && toc(t0) < 8
        pause(0.05);
    end
    pause(2);
    fprintf('%6.1f deg: M1 %.2f deg %.3f Nm | M2 %.2f deg %.3f Nm\n', deg, ...
        rad2deg(L.Latest.pos(1)), L.Latest.torque(1), rad2deg(L.Latest.pos(2)), L.Latest.torque(2));
end
L.pos(M, 0, 1.0, 3.0);
pause(4);
L.disable(M);
T = L.stopRecording('pos_test.csv');

figure;
plot(T.t_s - T.t_s(1), rad2deg([T.m1_pos T.m2_pos])); grid on
xlabel('t [s]'); ylabel('posisi [deg]'); legend('M1','M2');
