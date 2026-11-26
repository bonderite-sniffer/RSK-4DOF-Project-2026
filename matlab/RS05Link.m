classdef RS05Link < handle
%RS05LINK  USB-serial link to the RS05 HAL firmware (STM32H723, protocol v2, 4 motors).
%
%   Same binary protocol as python/rs05_host.py and Firmware/App/Inc/protocol.h:
%     A5 5A | type | len | payload | crc16 (lo,hi)   CRC-16/CCITT-FALSE
%   Payloads are little-endian. Units: rad, rad/s, Nm, A, degC.
%   Motors 1..4: M1, M2 on CAN1, M3, M4 on CAN2. Mask: bit0 = M1 ... bit3 = M4
%   (RS05Link.mask([1 3]) = 5, all = 15).
%
%   L = RS05Link("COM8");               % opens port, starts RX/heartbeat timer
%   info = L.ping();
%   L.pos(RS05Link.mask([1 2]), deg2rad(45), 1.0, 3.0);
%   L.zeroHere(15);                     % calib.c: current pose = 0 for all
%   L.Latest                            % last telemetry sample
%   delete(L)                           % E-STOP + close port
%
%   In Simulink (RS05Block) use RS05Link(port,"UseTimer",false) and call
%   poll()/heartbeat() from the block's step.

    properties (Constant)
        CMD_PING = 1; CMD_ESTOP = 2; CMD_HEARTBEAT = 3; CMD_SET_CFG = 5; CMD_SCAN = 6; CMD_GET_HEALTH = 7;
        CMD_DISABLE = 17; CMD_POS = 32; CMD_VEL = 33; CMD_MIT = 34;
        CMD_READ_PARAM = 48; CMD_WRITE_PARAM = 49; CMD_CALIB = 64;
        EVT_ACK = 128; EVT_TELEM = 129; EVT_SCAN = 130; EVT_PARAM = 131; EVT_EVENT = 132;
        EVT_PONG = 133; EVT_HEALTH = 134;
        CAL_ZERO_HERE = 1; CAL_HOME = 2; CAL_SET_CAN_ID = 3; CAL_SAVE = 4;
        NM = 4;    % motors
        CFG = struct('TSCALE',1,'TELEM_DIV',2,'SOFTLIM_EN',3,'POS_MIN',4,'POS_MAX',5, ...
                     'TEMP_MAX',6,'WD_MS',7,'HOST_ID',10,'SAFE_KP',11,'SAFE_KD',12,'MOTOR_ID0',16);
        MODE_NAMES = {'IDLE','MIT','VEL','POS','CALIB'};
        EVENT_NAMES = {'HOME_DONE','HOME_FAIL','SOFTLIMIT','OVERTEMP','FAULT','OFFLINE', ...
                       'WATCHDOG','ESTOP','CAN_ERROR','HOME_STALL','ZERO_DONE','ZERO_FAIL', ...
                       'ID_CHANGED','ID_FAIL'};
        FAULT_NAMES = {'undervoltage','overcurrent','overtemp','mag-encoder','HALL/encoder','uncalibrated'};
    end

    properties
        Port
        SP                      % serialport object
        Latest = []             % struct: last telemetry sample (fields are 1x4)
        Health = []             % struct: last CAN health frame
        OnEvent = []            % optional @(msgString) callback for events/errors
        EventLog = {}           % recent event strings (newest last)
        CrcErrors = 0
        HistN = 30000           % circular history length (samples)
    end

    properties (SetAccess = private)
        Hist                    % circular buffers, see allocHist
        HistIdx = 0
        HistCount = 0
        Recording = false
    end

    properties (Access = private)
        Buf = uint8([])
        Tmr = []
        Tick = 0
        Busy = false
        AckType = -1
        AckStatus = NaN
        Pong = []
        ScanResults = {}
        Param = []
        Events = zeros(0,4)     % [time motor code value]
        RecData = []
        RecN = 0
    end

    methods
        %% ------------------------------------------------------------ lifecycle
        function obj = RS05Link(port, opts)
            arguments
                port (1,1) string
                opts.Baud (1,1) double = 921600      % ignored by USB CDC, used by USART2 fallback
                opts.UseTimer (1,1) logical = true
            end
            obj.Port = port;
            obj.SP = serialport(port, opts.Baud, "Timeout", 0.05);
            flush(obj.SP);
            obj.allocHist();
            if opts.UseTimer
                obj.Tmr = timer('ExecutionMode','fixedSpacing','Period',0.01, ...
                    'BusyMode','drop','Name','RS05Link', ...
                    'TimerFcn',@(~,~) obj.service());
                start(obj.Tmr);
            end
        end

        function delete(obj)
            try
                if ~isempty(obj.Tmr) && isvalid(obj.Tmr)
                    stop(obj.Tmr); delete(obj.Tmr);
                end
            catch
            end
            try
                if ~isempty(obj.SP)
                    obj.estop();
                    pause(0.05);
                end
            catch
            end
            obj.SP = [];   % releases the port
        end

        %% ------------------------------------------------------------ io
        function service(obj)
            % Timer callback (100 Hz): read serial, heartbeat at 25 Hz.
            obj.Tick = obj.Tick + 1;
            try
                obj.poll();
                if mod(obj.Tick, 4) == 0
                    obj.heartbeat();
                end
            catch err
                obj.logEvent("link error: " + err.message);
            end
        end

        function heartbeat(obj)
            obj.sendFrame(obj.CMD_HEARTBEAT, uint8([]));
        end

        function poll(obj)
            % Read whatever arrived and dispatch complete frames.
            if obj.Busy || isempty(obj.SP), return; end
            obj.Busy = true;
            cleanup = onCleanup(@() obj.clearBusy()); %#ok<NASGU>
            n = obj.SP.NumBytesAvailable;
            if n > 0
                data = uint8(read(obj.SP, n, "uint8"));
                obj.Buf = [obj.Buf, reshape(data, 1, [])];
                obj.parseBuffer();
            end
        end

        function sendFrame(obj, type, payload)
            write(obj.SP, RS05Link.encodeFrame(type, payload), "uint8");
        end

        %% ------------------------------------------------------------ commands
        function info = ping(obj)
            obj.Pong = [];
            obj.sendFrame(obj.CMD_PING, uint8([]));
            obj.waitFor(@() ~isempty(obj.Pong), 1.0, "PING");
            v = obj.Pong.fw;
            info = struct('fw', sprintf('%d.%d.%d', bitshift(v,-16), bitand(bitshift(v,-8),255), bitand(v,255)), ...
                'ctrlHz', obj.Pong.hz, 'nMotors', obj.Pong.nm, 'nBus', obj.Pong.nb, 'proto', obj.Pong.proto);
        end

        function estop(obj)
            obj.sendFrame(obj.CMD_ESTOP, uint8([]));
        end

        function setCfg(obj, key, value, wait)
            if nargin < 4, wait = true; end
            obj.request(obj.CMD_SET_CFG, [uint8(obj.CFG.(key)), RS05Link.f32(value)], wait);
        end

        function setMotorId(obj, motor, canId)
            % Tell the firmware which CAN ID motor 1..4 has (does not change the motor).
            obj.request(obj.CMD_SET_CFG, [uint8(obj.CFG.MOTOR_ID0 + motor - 1), RS05Link.f32(canId)], true);
        end

        function res = scan(obj, busMask)
            % busMask: 1 = CAN1, 2 = CAN2, 3 = both. Returns cell of structs (bus, ids, busError).
            if nargin < 2, busMask = 3; end
            res = {};
            for b = 0:1
                if ~bitand(busMask, bitshift(1, b)), continue; end
                n0 = numel(obj.ScanResults);
                obj.sendFrame(obj.CMD_SCAN, uint8(bitshift(1, b)));
                obj.waitFor(@() numel(obj.ScanResults) > n0, 3.0, "SCAN");
                res{end+1} = obj.ScanResults{end}; %#ok<AGROW>
            end
        end

        function h = getHealth(obj)
            obj.Health = [];
            obj.sendFrame(obj.CMD_GET_HEALTH, uint8([]));
            obj.waitFor(@() ~isempty(obj.Health), 1.0, "HEALTH");
            h = obj.Health;
        end

        function disable(obj, mask, clearFault, wait)
            if nargin < 3, clearFault = false; end
            if nargin < 4, wait = true; end
            obj.request(obj.CMD_DISABLE, uint8([mask, clearFault]), wait);
        end

        function pos(obj, mask, target, velLim, curLim, wait)
            % CSP position: target [rad], velLim [rad/s], curLim [A]
            if nargin < 6, wait = true; end
            obj.request(obj.CMD_POS, [uint8(mask), RS05Link.f32([target velLim curLim])], wait);
        end

        function vel(obj, mask, v, curLim, acc, wait)
            % velocity: v [rad/s], curLim [A], acc [rad/s^2]
            if nargin < 6, wait = true; end
            obj.request(obj.CMD_VEL, [uint8(mask), RS05Link.f32([v curLim acc])], wait);
        end

        function mit(obj, mask, p, v, kp, kd, t, wait)
            % MIT: tau = kp*(p - pos) + kd*(v - vel) + t
            if nargin < 8, wait = true; end
            obj.request(obj.CMD_MIT, [uint8(mask), RS05Link.f32([p v kp kd t])], wait);
        end

        %% ---- calibration (calib.c on the STM32)
        function calib(obj, op, mask, a, b, c, d)
            if nargin < 4, a = 0; end
            if nargin < 5, b = 0; end
            if nargin < 6, c = 0; end
            if nargin < 7, d = 0; end
            obj.request(obj.CMD_CALIB, [uint8([op mask]), RS05Link.f32([a b c d])], true);
        end

        function ok = zeroHere(obj, mask, timeout)
            % Current pose of the motors in mask becomes 0. ok(k) true when ZERO_DONE arrived.
            if nargin < 3, timeout = 2.0; end
            t0 = posixtime(datetime('now'));
            obj.calib(obj.CAL_ZERO_HERE, mask);
            ok = false(1, obj.NM);
            for k = 1:obj.NM
                if bitand(mask, bitshift(1, k-1))
                    ok(k) = obj.waitEvent(11, k, timeout, t0);
                end
            end
        end

        function home(obj, mask, direction, speed, curLim, backoff)
            obj.calib(obj.CAL_HOME, mask, direction, speed, curLim, backoff);
        end

        function ok = setCanId(obj, motor, newId, timeout)
            % Change the CAN ID of ONE motor (1..4). Only that motor may answer on its old ID.
            if nargin < 4, timeout = 2.0; end
            t0 = posixtime(datetime('now'));
            obj.calib(obj.CAL_SET_CAN_ID, bitshift(1, motor-1), newId);
            ok = obj.waitEvent(13, motor, timeout, t0);
        end

        function saveParams(obj, mask)
            obj.calib(obj.CAL_SAVE, mask);
        end

        function found = waitEvent(obj, code, motor, timeout, since)
            % true if event `code` for motor 1..4 (0 = any) arrived after `since` (posix time)
            t0 = tic;
            found = false;
            while toc(t0) < timeout
                E = obj.Events;
                hit = E(:,1) >= since & E(:,3) == code;
                if motor > 0, hit = hit & E(:,2) == motor; end
                if any(hit), found = true; return; end
                obj.poll();
                pause(0.01);
            end
        end

        function value = readParam(obj, motor, index, asFloat)
            % motor 1..4
            if nargin < 4, asFloat = true; end
            obj.Param = [];
            obj.sendFrame(obj.CMD_READ_PARAM, [uint8(motor - 1), typecast(uint16(index),'uint8')]);
            obj.waitFor(@() ~isempty(obj.Param), 1.0, "READ_PARAM");
            if ~obj.Param.ok, error('RS05Link:param','param 0x%04X read failed', index); end
            if asFloat
                value = double(typecast(obj.Param.raw, 'single'));
            else
                value = double(obj.Param.raw);
            end
        end

        %% ------------------------------------------------------------ data
        function H = history(obj, seconds)
            % Telemetry of the last `seconds`, oldest first. Fields: t [s], pos/vel/torque/temp/iq (N x 4)
            n = obj.HistCount;
            if n == 0
                H = struct('t',zeros(0,1),'pos',zeros(0,4),'vel',zeros(0,4), ...
                           'torque',zeros(0,4),'temp',zeros(0,4),'iq',zeros(0,4));
                return
            end
            idx = mod((obj.HistIdx - n : obj.HistIdx - 1), obj.HistN) + 1;
            t = obj.Hist.t(idx);
            keep = t >= t(end) - seconds;
            idx = idx(keep);
            H.t = obj.Hist.t(idx);
            H.pos = obj.Hist.pos(idx,:);
            H.vel = obj.Hist.vel(idx,:);
            H.torque = obj.Hist.torque(idx,:);
            H.temp = obj.Hist.temp(idx,:);
            H.iq = obj.Hist.iq(idx,:);
        end

        function startRecording(obj)
            obj.RecData = zeros(20000, 39);
            obj.RecN = 0;
            obj.Recording = true;
        end

        function T = stopRecording(obj, csvFile)
            % Returns a table; also writes CSV if a file name is given.
            obj.Recording = false;
            names = {'t_s','seq'};
            fields = {'mode','fault','online','pos','vel','torque','temp','iq','sp'};
            for m = 1:4
                for f = 1:numel(fields)
                    names{end+1} = sprintf('m%d_%s', m, fields{f}); %#ok<AGROW>
                end
            end
            names{end+1} = 'host_t';
            T = array2table(obj.RecData(1:obj.RecN, :), 'VariableNames', names);
            if nargin > 1 && strlength(string(csvFile)) > 0
                writetable(T, csvFile);
            end
        end
    end

    methods (Access = private)
        function clearBusy(obj)
            obj.Busy = false;
        end

        function allocHist(obj)
            N = obj.HistN;
            obj.Hist = struct('t',zeros(N,1),'pos',zeros(N,4),'vel',zeros(N,4), ...
                'torque',zeros(N,4),'temp',zeros(N,4),'iq',zeros(N,4));
            obj.HistIdx = 0;
            obj.HistCount = 0;
        end

        function request(obj, type, payload, wait)
            if ~wait
                obj.sendFrame(type, payload);
                return
            end
            obj.AckType = type;
            obj.AckStatus = NaN;
            obj.sendFrame(type, payload);
            obj.waitFor(@() ~isnan(obj.AckStatus), 1.0, sprintf('cmd 0x%02X', type));
            st = obj.AckStatus;
            obj.AckType = -1;
            if st ~= 0
                error('RS05Link:rejected', 'cmd 0x%02X rejected: %s', type, RS05Link.ackName(st));
            end
        end

        function waitFor(obj, cond, timeout, what)
            t0 = tic;
            while ~cond()
                if toc(t0) > timeout
                    error('RS05Link:timeout', 'No reply to %s (firmware running? right port?)', what);
                end
                obj.poll();
                pause(0.003);
            end
        end

        function parseBuffer(obj)
            b = obj.Buf;
            i = 1;
            while true
                L = numel(b);
                if L - i + 1 < 2, break; end
                k = find(b(i:L-1) == 165 & b(i+1:L) == 90, 1);
                if isempty(k)
                    if b(L) == 165, i = L; else, i = L + 1; end
                    break
                end
                s = i + k - 1;
                if L - s + 1 < 4, i = s; break; end
                len = double(b(s+3));
                if len > 128, i = s + 2; continue; end
                if L - s + 1 < 6 + len, i = s; break; end
                body = b(s+2 : s+3+len);
                crc = double(b(s+4+len)) + 256 * double(b(s+5+len));
                if RS05Link.crc16(body) == crc
                    obj.dispatch(double(b(s+2)), b(s+4 : s+3+len));
                    i = s + 6 + len;
                else
                    obj.CrcErrors = obj.CrcErrors + 1;
                    i = s + 2;
                end
            end
            obj.Buf = b(i:end);
        end

        function dispatch(obj, type, p)
            switch type
                case obj.EVT_TELEM
                    if numel(p) == 120, obj.onTelem(p); end
                case obj.EVT_HEALTH
                    if numel(p) == 92, obj.onHealth(p); end
                case obj.EVT_ACK
                    cmd = double(p(1));
                    st = double(typecast(p(2), 'int8'));
                    if cmd == obj.AckType
                        obj.AckStatus = st;
                    elseif st ~= 0
                        obj.logEvent(sprintf('cmd 0x%02X rejected: %s', cmd, RS05Link.ackName(st)));
                    end
                case obj.EVT_EVENT
                    m = double(p(1));
                    code = double(p(2));
                    val = double(typecast(p(3:6), 'single'));
                    if m == 255, who = "ALL"; else, who = "M" + (m + 1); end
                    if code >= 1 && code <= numel(obj.EVENT_NAMES)
                        name = obj.EVENT_NAMES{code};
                    else
                        name = sprintf('event %d', code);
                    end
                    obj.Events(end+1,:) = [posixtime(datetime('now')), m + 1, code, val];
                    if size(obj.Events, 1) > 500, obj.Events(1,:) = []; end
                    obj.logEvent(sprintf('%s %s (value %.3f)', who, name, val));
                case obj.EVT_PONG
                    u = double(typecast(p(1:8), 'uint32'));
                    obj.Pong = struct('fw', u(1), 'hz', u(2), 'nm', double(p(9)), ...
                        'nb', double(p(10)), 'proto', double(p(11)));
                case obj.EVT_SCAN
                    n = double(p(3));
                    obj.ScanResults{end+1} = struct('bus', double(p(1)) + 1, ...
                        'busError', p(2) ~= 0, 'ids', double(p(4:3+n)));
                case obj.EVT_PARAM
                    obj.Param = struct('motor', double(p(1)), 'index', double(typecast(p(2:3),'uint16')), ...
                        'ok', p(4) ~= 0, 'raw', typecast(p(5:8), 'uint32'));
            end
        end

        function onTelem(obj, p)
            T.t = double(typecast(p(1:4), 'uint32')) / 1000;
            T.seq = double(typecast(p(5:6), 'uint16'));
            T.hostOk = p(7) ~= 0;
            for m = 1:4
                o = 8 + (m - 1) * 28;
                h = double(p(o+1:o+4));
                f = double(typecast(p(o+5:o+28), 'single'));
                T.mode(m) = h(1);   T.rsMode(m) = h(2);  T.fault(m) = h(3);
                T.online(m) = bitand(h(4), 1) ~= 0;
                T.softLimited(m) = bitand(h(4), 2) ~= 0;
                T.pos(m) = f(1);    T.vel(m) = f(2);     T.torque(m) = f(3);
                T.temp(m) = f(4);   T.iq(m) = f(5);      T.sp(m) = f(6);
            end
            obj.Latest = T;

            k = mod(obj.HistIdx, obj.HistN) + 1;
            obj.Hist.t(k) = T.t;
            obj.Hist.pos(k,:) = T.pos;
            obj.Hist.vel(k,:) = T.vel;
            obj.Hist.torque(k,:) = T.torque;
            obj.Hist.temp(k,:) = T.temp;
            obj.Hist.iq(k,:) = T.iq;
            obj.HistIdx = k;
            obj.HistCount = min(obj.HistCount + 1, obj.HistN);

            if obj.Recording
                if obj.RecN >= size(obj.RecData, 1)
                    obj.RecData = [obj.RecData; zeros(20000, 39)];
                end
                row = [T.t, T.seq];
                for m = 1:4
                    row = [row, T.mode(m), T.fault(m), T.online(m), T.pos(m), T.vel(m), ...
                           T.torque(m), T.temp(m), T.iq(m), T.sp(m)]; %#ok<AGROW>
                end
                obj.RecN = obj.RecN + 1;
                obj.RecData(obj.RecN, :) = [row, posixtime(datetime('now'))];
            end
        end

        function onHealth(obj, p)
            H.t = double(typecast(p(1:4), 'uint32')) / 1000;
            for b = 1:2
                o = 4 + (b - 1) * 32;
                c = double(typecast(p(o+5:o+32), 'uint32'));
                H.bus(b) = struct('initOk', p(o+1) ~= 0, 'eflg', double(p(o+2)), 'tec', double(p(o+3)), ...
                    'rec', double(p(o+4)), 'txSent', c(1), 'txFail', c(2), 'txBusy', c(3), ...
                    'rxCount', c(4), 'rxOvf', c(5), 'busOff', c(6), 'spiErr', c(7));
            end
            H.vbus = double(typecast(p(69:84), 'single'));
            H.fbAgeMs = double(typecast(p(85:92), 'uint16'));
            obj.Health = H;
        end

        function logEvent(obj, msg)
            msg = string(datetime('now','Format','HH:mm:ss')) + "  " + string(msg);
            obj.EventLog{end+1} = char(msg);
            if numel(obj.EventLog) > 200, obj.EventLog(1) = []; end
            if ~isempty(obj.OnEvent)
                try, obj.OnEvent(msg); catch, end
            else
                fprintf('[RS05] %s\n', msg);
            end
        end
    end

    methods (Static)
        function m = mask(motors)
            % RS05Link.mask([1 3]) -> 5. Motors 1..4.
            m = 0;
            for k = motors(:)'
                m = bitor(m, bitshift(1, k - 1));
            end
        end

        function f = encodeFrame(type, payload)
            payload = reshape(uint8(payload), 1, []);
            body = [uint8(type), uint8(numel(payload)), payload];
            c = RS05Link.crc16(body);
            f = [uint8([165 90]), body, uint8(bitand(c, 255)), uint8(bitshift(c, -8))];
        end

        function c = crc16(data)
            % CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), table driven.
            persistent T
            if isempty(T)
                T = zeros(1, 256, 'uint16');
                for n = 0:255
                    r = bitshift(uint16(n), 8);
                    for k = 1:8
                        if bitand(r, uint16(32768))
                            r = bitxor(bitshift(r, 1), uint16(4129));
                        else
                            r = bitshift(r, 1);
                        end
                    end
                    T(n + 1) = r;
                end
            end
            c = uint16(65535);
            for k = 1:numel(data)
                idx = bitxor(bitshift(c, -8), uint16(data(k)));
                c = bitxor(bitshift(c, 8), T(double(idx) + 1));
            end
            c = double(c);
        end

        function b = f32(x)
            b = typecast(single(reshape(x, 1, [])), 'uint8');
        end

        function s = ackName(st)
            names = containers.Map({0,-1,-2,-3,-4,-5,-6}, ...
                {'OK','BAD_LEN','BUSY (motor not idle)','QUEUE_FULL','BAD_ARG','CAN_ERR','UNKNOWN_CMD'});
            if isKey(names, st), s = names(st); else, s = sprintf('%d', st); end
        end

        function s = faultString(bits)
            s = '';
            for b = 0:5
                if bitand(bits, bitshift(1, b))
                    s = [s, RS05Link.FAULT_NAMES{b+1}, ' ']; %#ok<AGROW>
                end
            end
            if isempty(s), s = '-'; else, s = strtrim(s); end
        end
    end
end
