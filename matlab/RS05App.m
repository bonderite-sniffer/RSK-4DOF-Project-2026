classdef RS05App < handle
%RS05APP  GUI for 4x RobStride EduLite 05 (M1,M2 on CAN1 / M3,M4 on CAN2) via the
%STM32H723 HAL firmware (protocol v2).
%
%   >> app = RS05App;
%
%   1. COM port -> Connect (the USB-C port of the board)
%   2. tick the motors to actuate (M1..M4)
%   3. pick a mode tab (MIT / Velocity / Position), set values, "Kirim command"
%   4. Zeroing (calib.c on the STM32), CAN IDs, safety & torque scale on the left
%   5. live telemetry on the right: position rel. zero, speed, torque, temperature
%   E-STOP stops everything; closing the window sends E-STOP too.

    properties
        Link = []
        Fig
        PortDD
        ConnBtn
        StatusLbl
        MotorCB              % 1x4 checkboxes
        IdFld                % 1x4 CAN ID fields
        Tabs
        TabMIT
        TabVEL
        TabPOS
        MitP
        MitUseCur
        MitV
        MitKp
        MitKd
        MitT
        VelRpm
        VelCur
        VelAcc
        PosDeg
        PosVlim
        PosCur
        NewIdMotor
        NewIdFld
        TscaleFld
        SoftMinFld
        SoftMaxFld
        RecBtn
        WinDD
        Tbl
        HealthLbl
        Ax
        Lines
        LogArea
        UiTimer = []
    end

    properties (Constant)
        COLORS = [0.90 0.15 0.12; 0.04 0.36 1.00; 0.00 0.63 0.35; 1.00 0.71 0.00];
    end

    methods
        function app = RS05App()
            app.buildUi();
            app.refreshPorts();
        end

        %% ============================================================ UI
        function buildUi(app)
            app.Fig = uifigure('Name', 'RS05 x4 - MATLAB', 'Position', [30 30 1420 880]);
            app.Fig.CloseRequestFcn = @(~,~) app.onClose();
            g = uigridlayout(app.Fig, [1 2]);
            g.ColumnWidth = {400, '1x'};

            L = uigridlayout(g, [6 1]);
            L.RowHeight = {96, 112, 300, 120, 100, 56};
            L.Padding = [0 0 0 0];
            L.RowSpacing = 8;
            grey = [0.45 0.45 0.45];

            % 1. connection
            p1 = uipanel(L, 'Title', '1. Koneksi USB serial (USB-C board)');
            g1 = uigridlayout(p1, [2 3]);
            g1.ColumnWidth = {'1x', 70, 100};
            g1.RowHeight = {28, 22};
            app.PortDD = uidropdown(g1, 'Items', {'-'});
            uibutton(g1, 'Text', 'Refresh', 'ButtonPushedFcn', @(~,~) app.refreshPorts());
            app.ConnBtn = uibutton(g1, 'Text', 'Connect', 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.safe(@() app.toggleConnect()));
            app.StatusLbl = uilabel(g1, 'Text', 'Disconnected', 'FontColor', grey);
            app.StatusLbl.Layout.Row = 2; app.StatusLbl.Layout.Column = [1 3];

            % 2. motors + CAN IDs
            p2 = uipanel(L, 'Title', '2. Motor yang diaktuasi & CAN ID');
            g2 = uigridlayout(p2, [3 5]);
            g2.ColumnWidth = {70, '1x', '1x', '1x', '1x'};
            g2.RowHeight = {22, 26, 26};
            uilabel(g2, 'Text', '');
            names = {'M1 (CAN1)', 'M2 (CAN1)', 'M3 (CAN2)', 'M4 (CAN2)'};
            for k = 1:4
                l = uilabel(g2, 'Text', names{k}, 'FontWeight', 'bold', 'FontColor', app.COLORS(k,:));
                l.Layout.Row = 1; l.Layout.Column = k + 1;
            end
            l = uilabel(g2, 'Text', 'Aktuasi'); l.Layout.Row = 2; l.Layout.Column = 1;
            l = uilabel(g2, 'Text', 'CAN ID');  l.Layout.Row = 3; l.Layout.Column = 1;
            defIds = [1 2 1 2];
            for k = 1:4
                cb = uicheckbox(g2, 'Text', '', 'Value', k == 1);
                cb.Layout.Row = 2; cb.Layout.Column = k + 1;
                app.MotorCB{k} = cb;
                f = uieditfield(g2, 'numeric', 'Value', defIds(k), 'Limits', [1 127], ...
                    'RoundFractionalValues', 'on', ...
                    'ValueChangedFcn', @(~,~) app.safe(@() app.pushIds()));
                f.Layout.Row = 3; f.Layout.Column = k + 1;
                app.IdFld{k} = f;
            end

            % 3. mode + values
            p3 = uipanel(L, 'Title', '3. Mode kontrol & nilai');
            g3 = uigridlayout(p3, [2 2]);
            g3.RowHeight = {'1x', 34};
            app.Tabs = uitabgroup(g3);
            app.Tabs.Layout.Row = 1; app.Tabs.Layout.Column = [1 2];

            app.TabMIT = uitab(app.Tabs, 'Title', 'MIT');
            gm = uigridlayout(app.TabMIT, [6 2]);
            gm.ColumnWidth = {150, '1x'};
            gm.RowHeight = repmat({26}, 1, 6);
            app.MitP  = app.numRow(gm, 1, 'p_des [deg]', 0, [-720 720]);
            app.MitUseCur = uicheckbox(gm, 'Text', 'p_des = posisi sekarang (anti lompat)', 'Value', true);
            app.MitUseCur.Layout.Row = 2; app.MitUseCur.Layout.Column = [1 2];
            app.MitV  = app.numRow(gm, 3, 'v_des [rad/s]', 0, [-50 50]);
            app.MitKp = app.numRow(gm, 4, 'Kp [Nm/rad]', 5, [0 500]);
            app.MitKd = app.numRow(gm, 5, 'Kd [Nm s/rad]', 0.3, [0 5]);
            app.MitT  = app.numRow(gm, 6, 'tau_ff [Nm]', 0, [-6 6]);

            app.TabVEL = uitab(app.Tabs, 'Title', 'Velocity');
            gv = uigridlayout(app.TabVEL, [4 2]);
            gv.ColumnWidth = {150, '1x'};
            gv.RowHeight = repmat({26}, 1, 4);
            app.VelRpm = app.numRow(gv, 1, 'Kecepatan [rpm]', 10, [-477 477]);
            app.VelCur = app.numRow(gv, 2, 'Limit arus [A]', 3, [0.1 11]);
            app.VelAcc = app.numRow(gv, 3, 'Akselerasi [rad/s^2]', 10, [0.1 200]);
            l = uilabel(gv, 'Text', 'Berputar terus. Soft limit menghentikan motor di batas.', ...
                'WordWrap', 'on', 'FontColor', grey);
            l.Layout.Row = 4; l.Layout.Column = [1 2];

            app.TabPOS = uitab(app.Tabs, 'Title', 'Position');
            gp = uigridlayout(app.TabPOS, [4 2]);
            gp.ColumnWidth = {150, '1x'};
            gp.RowHeight = repmat({26}, 1, 4);
            app.PosDeg  = app.numRow(gp, 1, 'Target [deg]', 0, [-720 720]);
            app.PosVlim = app.numRow(gp, 2, 'Limit kecepatan [rad/s]', 1, [0.05 50]);
            app.PosCur  = app.numRow(gp, 3, 'Limit arus [A]', 3, [0.1 11]);
            l = uilabel(gp, 'Text', 'Mode CSP: ke target dengan kecepatan dibatasi.', ...
                'WordWrap', 'on', 'FontColor', grey);
            l.Layout.Row = 4; l.Layout.Column = [1 2];

            bs = uibutton(g3, 'Text', 'Kirim command', 'FontWeight', 'bold', ...
                'BackgroundColor', [0 0.63 0.35], 'FontColor', [1 1 1], ...
                'ButtonPushedFcn', @(~,~) app.safe(@() app.sendCommand()));
            bs.Layout.Row = 2; bs.Layout.Column = 1;
            bd = uibutton(g3, 'Text', 'Stop / disable', ...
                'ButtonPushedFcn', @(~,~) app.safe(@() app.disableSel(false)));
            bd.Layout.Row = 2; bd.Layout.Column = 2;

            % 4. zeroing / calibration
            p4 = uipanel(L, 'Title', '4. Zeroing & kalibrasi (calib.c)');
            g4 = uigridlayout(p4, [3 4]);
            g4.RowHeight = {30, 28, 22};
            g4.ColumnWidth = {'1x', 70, 70, '1x'};
            b = uibutton(g4, 'Text', 'Set zero di posisi sekarang', ...
                'ButtonPushedFcn', @(~,~) app.safe(@() app.setZero()));
            b.Layout.Row = 1; b.Layout.Column = [1 2];
            b = uibutton(g4, 'Text', 'Clear fault', ...
                'ButtonPushedFcn', @(~,~) app.safe(@() app.disableSel(true)));
            b.Layout.Row = 1; b.Layout.Column = [3 4];
            l = uilabel(g4, 'Text', 'Ganti CAN ID:'); l.Layout.Row = 2; l.Layout.Column = 1;
            app.NewIdMotor = uidropdown(g4, 'Items', {'M1','M2','M3','M4'}, 'ItemsData', {1,2,3,4});
            app.NewIdMotor.Layout.Row = 2; app.NewIdMotor.Layout.Column = 2;
            app.NewIdFld = uieditfield(g4, 'numeric', 'Value', 2, 'Limits', [1 127], 'RoundFractionalValues', 'on');
            app.NewIdFld.Layout.Row = 2; app.NewIdFld.Layout.Column = 3;
            b = uibutton(g4, 'Text', 'Set ID', 'ButtonPushedFcn', @(~,~) app.safe(@() app.setCanId()));
            b.Layout.Row = 2; b.Layout.Column = 4;
            l = uilabel(g4, 'Text', 'Zero: disable, posisikan lengan, lalu Set zero. Set ID: 1 motor saja di bus.', ...
                'FontColor', grey);
            l.Layout.Row = 3; l.Layout.Column = [1 4];

            % 5. safety
            p5 = uipanel(L, 'Title', '5. Safety & skala');
            g5 = uigridlayout(p5, [2 5]);
            g5.ColumnWidth = {80, '1x', 60, '1x', 90};
            g5.RowHeight = {24, 28};
            uilabel(g5, 'Text', 'Soft [deg]');
            app.SoftMinFld = uieditfield(g5, 'numeric', 'Value', -110, 'Limits', [-180 0]);
            uilabel(g5, 'Text', 's/d');
            app.SoftMaxFld = uieditfield(g5, 'numeric', 'Value', 110, 'Limits', [0 180]);
            uilabel(g5, 'Text', '');
            uilabel(g5, 'Text', 'Tscale +-');
            app.TscaleFld = uieditfield(g5, 'numeric', 'Value', 5.5, 'Limits', [1 200], ...
                'Tooltip', 'Skala field torsi type 1. RS05 = 5.5 Nm (manual).');
            l = uilabel(g5, 'Text', 'Nm', 'FontColor', grey); l.Layout.Column = 3;
            b = uibutton(g5, 'Text', 'Apply', 'ButtonPushedFcn', @(~,~) app.safe(@() app.applySettings()));
            b.Layout.Row = 2; b.Layout.Column = [4 5];

            % 6. E-STOP
            uibutton(L, 'Text', 'E-STOP', 'FontSize', 20, 'FontWeight', 'bold', ...
                'BackgroundColor', [0.9 0.15 0.12], 'FontColor', [1 1 1], ...
                'ButtonPushedFcn', @(~,~) app.estop());

            % ---------------- right: telemetry
            R = uigridlayout(g, [4 1]);
            R.RowHeight = {128, 22, '1x', 130};
            R.Padding = [0 0 0 0];
            app.Tbl = uitable(R, 'ColumnName', {'Motor','Online','Mode','Posisi [deg]', ...
                'Speed [rpm]','Torsi [Nm]','Suhu [C]','iq [A]','VBUS [V]','Fault'}, ...
                'RowName', {}, 'Data', repmat({'-'}, 4, 10));
            app.HealthLbl = uilabel(R, 'Text', 'CAN health: -', 'FontName', 'Consolas');

            ga = uigridlayout(R, [2 2]);
            ga.Padding = [0 0 0 0];
            titles = {'Torsi [Nm]', 'Speed [rpm]', 'Posisi relatif zero [deg]', 'Temperatur [C]'};
            app.Lines = gobjects(4, 4);
            app.Ax = gobjects(1, 4);
            for a = 1:4
                ax = uiaxes(ga);
                title(ax, titles{a});
                xlabel(ax, 't [s]');
                grid(ax, 'on');
                hold(ax, 'on');
                for k = 1:4
                    app.Lines(a,k) = plot(ax, NaN, NaN, '-', 'LineWidth', 1.5, 'Color', app.COLORS(k,:));
                end
                legend(ax, {'M1','M2','M3','M4'}, 'Location', 'northwest', 'NumColumns', 4);
                app.Ax(a) = ax;
            end

            gb = uigridlayout(R, [1 2]);
            gb.ColumnWidth = {'1x', 170};
            gb.Padding = [0 0 0 0];
            app.LogArea = uitextarea(gb, 'Editable', 'off', 'Value', {'Event log'});
            gc = uigridlayout(gb, [3 1]);
            gc.RowHeight = {30, 24, 30};
            app.RecBtn = uibutton(gc, 'state', 'Text', 'Record CSV', ...
                'ValueChangedFcn', @(~,~) app.safe(@() app.toggleRecord()));
            uilabel(gc, 'Text', 'Jendela plot:');
            app.WinDD = uidropdown(gc, 'Items', {'10 s','30 s','60 s'}, 'ItemsData', {10, 30, 60});
        end

        function f = numRow(~, g, row, label, value, lim)
            l = uilabel(g, 'Text', label);
            l.Layout.Row = row; l.Layout.Column = 1;
            f = uieditfield(g, 'numeric', 'Value', value, 'Limits', lim);
            f.Layout.Row = row; f.Layout.Column = 2;
        end

        %% ============================================================ actions
        function refreshPorts(app)
            ports = serialportlist("available");
            if isempty(ports), ports = "-"; end
            app.PortDD.Items = cellstr(ports);
        end

        function toggleConnect(app)
            if isempty(app.Link)
                port = string(app.PortDD.Value);
                if port == "-", error('Pilih COM port dulu (Refresh).'); end
                app.Link = RS05Link(port);
                app.Link.OnEvent = @(m) app.log(m);
                try
                    info = app.Link.ping();
                catch err
                    delete(app.Link); app.Link = [];
                    rethrow(err);
                end
                app.Link.setCfg('TELEM_DIV', max(1, round(info.ctrlHz / 100)));   % ~100 Hz
                app.Link.setCfg('WD_MS', 1000);   % MATLAB timers can jitter
                app.pushIds();
                app.applySettings();
                app.StatusLbl.Text = sprintf('Connected %s | fw %s | %d Hz | %d motor', ...
                    port, info.fw, info.ctrlHz, info.nMotors);
                app.StatusLbl.FontColor = [0 0.55 0.3];
                app.ConnBtn.Text = 'Disconnect';
                app.UiTimer = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.1, ...
                    'BusyMode', 'drop', 'Name', 'RS05App', 'TimerFcn', @(~,~) app.refresh());
                start(app.UiTimer);
                app.log("connected " + port);
            else
                app.stopUiTimer();
                delete(app.Link);
                app.Link = [];
                app.ConnBtn.Text = 'Connect';
                app.StatusLbl.Text = 'Disconnected';
                app.StatusLbl.FontColor = [0.45 0.45 0.45];
                app.log("disconnected");
            end
        end

        function m = mask(app)
            m = 0;
            for k = 1:4
                if app.MotorCB{k}.Value, m = bitor(m, bitshift(1, k-1)); end
            end
            if m == 0, error('Centang minimal satu motor.'); end
        end

        function pushIds(app)
            if isempty(app.Link), return; end
            for k = 1:4
                app.Link.setMotorId(k, app.IdFld{k}.Value);
            end
        end

        function sendCommand(app)
            app.requireLink();
            m = app.mask();
            tab = app.Tabs.SelectedTab;
            if tab == app.TabMIT
                v = app.MitV.Value; kp = app.MitKp.Value; kd = app.MitKd.Value; t = app.MitT.Value;
                T = app.Link.Latest;
                for k = 1:4
                    bit = bitshift(1, k - 1);
                    if ~bitand(m, bit), continue; end
                    if app.MitUseCur.Value
                        p = T.pos(k);
                    else
                        p = deg2rad(app.MitP.Value);
                        if kp > 0 && abs(p - T.pos(k)) > deg2rad(30)
                            sel = uiconfirm(app.Fig, sprintf('M%d akan ditarik %.0f deg dengan Kp=%.1f. Lanjut?', ...
                                k, rad2deg(p - T.pos(k)), kp), 'MIT', 'Options', {'Lanjut','Batal'}, ...
                                'DefaultOption', 2, 'CancelOption', 2);
                            if ~strcmp(sel, 'Lanjut'), continue; end
                        end
                    end
                    app.Link.mit(bit, p, v, kp, kd, t);
                    app.log(sprintf('M%d MIT p=%.1f deg Kp=%.1f Kd=%.2f tff=%.2f', k, rad2deg(p), kp, kd, t));
                end
            elseif tab == app.TabVEL
                app.Link.vel(m, app.VelRpm.Value * 2 * pi / 60, app.VelCur.Value, app.VelAcc.Value);
                app.log(sprintf('mask %d VEL %.1f rpm', m, app.VelRpm.Value));
            else
                app.Link.pos(m, deg2rad(app.PosDeg.Value), app.PosVlim.Value, app.PosCur.Value);
                app.log(sprintf('mask %d POS %.1f deg', m, app.PosDeg.Value));
            end
        end

        function disableSel(app, clearFault)
            app.requireLink();
            app.Link.disable(app.mask(), clearFault);
            if clearFault
                app.log(sprintf('mask %d disable + clear fault', app.mask()));
            else
                app.log(sprintf('mask %d disable', app.mask()));
            end
        end

        function setZero(app)
            app.requireLink();
            m = app.mask();
            sel = uiconfirm(app.Fig, ['Motor tercentang di-DISABLE (lengan bisa jatuh), lalu posisi ' ...
                'sekarang dijadikan 0 deg. Lanjut?'], 'Set zero', ...
                'Options', {'Set zero','Batal'}, 'DefaultOption', 2, 'CancelOption', 2);
            if ~strcmp(sel, 'Set zero'), return; end
            app.Link.disable(m, false);
            pause(0.2);
            ok = app.Link.zeroHere(m);
            for k = find(bitand(m, bitshift(1, 0:3)))
                if ok(k)
                    app.log(sprintf('M%d zero OK', k));
                else
                    app.log(sprintf('M%d zero GAGAL (posisi tidak ~0 setelah zero)', k));
                end
            end
        end

        function setCanId(app)
            app.requireLink();
            k = app.NewIdMotor.Value;
            newId = app.NewIdFld.Value;
            sel = uiconfirm(app.Fig, sprintf(['Ganti CAN ID M%d dari %d ke %d? Hanya motor ini ' ...
                'yang boleh ada di bus dengan ID lama.'], k, app.IdFld{k}.Value, newId), 'Set CAN ID', ...
                'Options', {'Ganti','Batal'}, 'DefaultOption', 2, 'CancelOption', 2);
            if ~strcmp(sel, 'Ganti'), return; end
            app.Link.disable(15, false);
            if app.Link.setCanId(k, newId)
                app.IdFld{k}.Value = newId;
                app.log(sprintf('M%d CAN ID -> %d OK', k, newId));
            else
                app.log(sprintf('M%d CAN ID -> %d GAGAL (motor tidak menjawab)', k, newId));
            end
        end

        function applySettings(app)
            app.requireLink();
            app.Link.setCfg('TSCALE', app.TscaleFld.Value);
            app.Link.setCfg('POS_MIN', deg2rad(app.SoftMinFld.Value));
            app.Link.setCfg('POS_MAX', deg2rad(app.SoftMaxFld.Value));
            app.Link.setCfg('SOFTLIM_EN', 1);
            app.log(sprintf('settings: soft [%g, %g] deg, tscale %g', ...
                app.SoftMinFld.Value, app.SoftMaxFld.Value, app.TscaleFld.Value));
        end

        function estop(app)
            if ~isempty(app.Link)
                try, app.Link.estop(); catch, end
            end
            app.log("E-STOP");
        end

        function toggleRecord(app)
            app.requireLink();
            if app.RecBtn.Value
                app.Link.startRecording();
                app.RecBtn.Text = 'Stop & save';
                app.log("recording...");
            else
                [f, d] = uiputfile('*.csv', 'Simpan log', ['rs05_' char(datetime('now','Format','yyyyMMdd_HHmmss')) '.csv']);
                if isequal(f, 0)
                    T = app.Link.stopRecording();
                else
                    T = app.Link.stopRecording(fullfile(d, f));
                end
                app.RecBtn.Text = 'Record CSV';
                app.log(sprintf('recorded %d samples', height(T)));
                assignin('base', 'rs05_log', T);
            end
        end

        %% ============================================================ live view
        function refresh(app)
            if isempty(app.Link) || ~isvalid(app.Fig), return; end
            T = app.Link.Latest;
            if isempty(T), return; end
            win = app.WinDD.Value;
            H = app.Link.history(win);
            if numel(H.t) > 1
                t = H.t - H.t(end);
                data = {H.torque, H.vel * 60 / (2*pi), rad2deg(H.pos), H.temp};
                for a = 1:4
                    for k = 1:4
                        set(app.Lines(a,k), 'XData', t, 'YData', data{a}(:,k));
                    end
                    app.Ax(a).XLim = [-win 0];
                end
            end
            vb = nan(1, 4);
            if ~isempty(app.Link.Health), vb = app.Link.Health.vbus; end
            rows = cell(4, 10);
            for k = 1:4
                rows(k,:) = {sprintf('M%d', k), app.yesno(T.online(k)), RS05Link.MODE_NAMES{T.mode(k) + 1}, ...
                    sprintf('%.2f', rad2deg(T.pos(k))), sprintf('%.1f', T.vel(k) * 60 / (2*pi)), ...
                    sprintf('%.3f', T.torque(k)), sprintf('%.1f', T.temp(k)), sprintf('%.2f', T.iq(k)), ...
                    sprintf('%.1f', vb(k)), RS05Link.faultString(T.fault(k))};
            end
            app.Tbl.Data = rows;
            if ~isempty(app.Link.Health)
                s = '';
                for b = 1:2
                    B = app.Link.Health.bus(b);
                    s = [s, sprintf('CAN%d %s TEC %3d REC %3d fail %d ovf %d busoff %d   ', b, ...
                        app.yesno(B.initOk), B.tec, B.rec, B.txFail, B.rxOvf, B.busOff)]; %#ok<AGROW>
                end
                app.HealthLbl.Text = s;
            end
        end

        function log(app, msg)
            if isempty(app.LogArea) || ~isvalid(app.LogArea), return; end
            line = char(string(msg));
            if ~startsWith(line, digitsPattern(2) + ":")
                line = [char(datetime('now','Format','HH:mm:ss')) '  ' line];
            end
            v = app.LogArea.Value;
            v{end+1} = line;
            if numel(v) > 200, v = v(end-199:end); end
            app.LogArea.Value = v;
            try, scroll(app.LogArea, 'bottom'); catch, end
        end

        %% ============================================================ helpers
        function safe(app, fn)
            try
                fn();
            catch err
                uialert(app.Fig, err.message, 'Error');
                app.log("ERROR: " + err.message);
            end
        end

        function requireLink(app)
            if isempty(app.Link), error('Belum connect.'); end
        end

        function s = yesno(~, b)
            if b, s = 'ON'; else, s = 'off'; end
        end

        function stopUiTimer(app)
            if ~isempty(app.UiTimer) && isvalid(app.UiTimer)
                stop(app.UiTimer); delete(app.UiTimer);
            end
            app.UiTimer = [];
        end

        function onClose(app)
            app.stopUiTimer();
            if ~isempty(app.Link)
                delete(app.Link);
                app.Link = [];
            end
            delete(app.Fig);
        end
    end
end
