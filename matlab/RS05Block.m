classdef RS05Block < matlab.System
%RS05BLOCK  Simulink MATLAB System block: talks to the RS05 bench firmware over USB serial.
%
%   Protocol v2 / 4 motors (M1,M2 on CAN1, M3,M4 on CAN2).
%   Runs in "Interpreted execution" (serialport cannot be code-generated),
%   so no STM32 blockset / Embedded Coder is needed. Use with Simulation
%   Pacing = 1 so simulation time follows wall-clock time.
%
%   Inputs  (units chosen to match the GUI):
%     1 enable  : 1 = drive, 0 = disable motors
%     2 mode    : 1 = MIT, 2 = Velocity, 3 = Position (CSP)
%     3 mask    : bit mask, 1 = M1, 2 = M2, 4 = M3, 8 = M4 (15 = all)
%     4 mit     : [p_des_deg  v_des_rad_s  Kp  Kd  tau_ff_Nm]
%     5 vel     : [speed_rpm  cur_lim_A  acc_rad_s2]
%     6 pos     : [target_deg  vel_lim_rad_s  cur_lim_A]
%   Outputs (4x1, [M1; M2; M3; M4]):
%     pos_deg (relative to zero), vel_rpm, torque_Nm, temp_C, iq_A
%
%   A command is sent only when an input changes (max every MinCmdSteps
%   steps), so a Sine Wave on the position target streams a trajectory.

    properties (Nontunable)
        Port = 'COM5'            % serial port (Windows: 'COMx', Linux: '/dev/ttyACM0')
        SampleTime = 0.01        % block step [s]
        TorqueScale = 5.5        % type-1 torque field range +-Nm (RS05 = 5.5)
        SoftMinDeg = -110        % soft limit [deg]
        SoftMaxDeg = 110
        MinCmdSteps = 2          % rate limit for commands (2 x 10 ms = 50 Hz)
    end

    properties (Access = private)
        L = []
        K = 0
        LastCmd = []
        LastSendK = -inf
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.L = RS05Link(string(obj.Port), 'UseTimer', false);
            info = obj.L.ping();
            fprintf('[RS05Block] connected %s, fw %s, loop %d Hz, %d motors\n', obj.Port, info.fw, info.ctrlHz, info.nMotors);
            obj.L.setCfg('TELEM_DIV', max(1, round(info.ctrlHz * obj.SampleTime)));  % one sample per step
            obj.L.setCfg('WD_MS', 1000);
            obj.L.setCfg('TSCALE', obj.TorqueScale);
            obj.L.setCfg('POS_MIN', deg2rad(obj.SoftMinDeg));
            obj.L.setCfg('POS_MAX', deg2rad(obj.SoftMaxDeg));
            obj.L.setCfg('SOFTLIM_EN', 1);
        end

        function [pos, vel, tor, temp, iq] = stepImpl(obj, en, mode, mask, mitp, velp, posp)
            L = obj.L;
            L.poll();
            obj.K = obj.K + 1;
            if mod(obj.K, max(1, round(0.04 / obj.SampleTime))) == 0
                L.heartbeat();                               % ~25 Hz
            end

            cur = [double(en), double(mode), double(mask), ...
                   reshape(double(mitp),1,[]), reshape(double(velp),1,[]), reshape(double(posp),1,[])];
            if ~isequal(cur, obj.LastCmd) && (obj.K - obj.LastSendK) >= obj.MinCmdSteps
                mk = max(1, min(15, round(double(mask))));
                if en > 0.5
                    switch round(double(mode))
                        case 1
                            L.mit(mk, deg2rad(mitp(1)), mitp(2), mitp(3), mitp(4), mitp(5), false);
                        case 2
                            L.vel(mk, velp(1) * 2 * pi / 60, velp(2), velp(3), false);
                        case 3
                            L.pos(mk, deg2rad(posp(1)), posp(2), posp(3), false);
                    end
                else
                    L.disable(mk, false, false);
                end
                obj.LastCmd = cur;
                obj.LastSendK = obj.K;
            end

            T = L.Latest;
            if isempty(T)
                pos = zeros(4,1); vel = zeros(4,1); tor = zeros(4,1); temp = zeros(4,1); iq = zeros(4,1);
            else
                pos = rad2deg(T.pos(:));
                vel = T.vel(:) * 60 / (2*pi);
                tor = T.torque(:);
                temp = T.temp(:);
                iq = T.iq(:);
            end
        end

        function releaseImpl(obj)
            if ~isempty(obj.L)
                delete(obj.L);    % sends E-STOP and closes the port
                obj.L = [];
            end
        end

        function resetImpl(obj)
            obj.K = 0;
            obj.LastCmd = [];
            obj.LastSendK = -inf;
        end

        %% ---- block interface (so Simulink never has to run stepImpl to find sizes)
        function n = getNumInputsImpl(~),  n = 6; end
        function n = getNumOutputsImpl(~), n = 5; end
        function varargout = getOutputSizeImpl(~),      varargout = repmat({[4 1]}, 1, 5); end
        function varargout = getOutputDataTypeImpl(~),  varargout = repmat({'double'}, 1, 5); end
        function varargout = isOutputComplexImpl(~),    varargout = repmat({false}, 1, 5); end
        function varargout = isOutputFixedSizeImpl(~),  varargout = repmat({true}, 1, 5); end
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end
        function varargout = getInputNamesImpl(~)
            varargout = {'enable', 'mode', 'mask', 'mit', 'vel', 'pos'};
        end
        function varargout = getOutputNamesImpl(~)
            varargout = {'pos_deg', 'vel_rpm', 'torque_Nm', 'temp_C', 'iq_A'};
        end
        function icon = getIconImpl(~)
            icon = sprintf('RS05 x4\nUSB serial');
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end
        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
