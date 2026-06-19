% Kinematics for Robotics
% 3 Degree Of Freedom Robot Arm Simulation

% Features include:
%   Proof of workspace coverage (all chessboard grid locations)
%   Waypoint inputs through file or Manual input
%   Geometric inverse kinematics (elbow-up pose)
%   Linear interpolations of Cartesian path
%   PID joint controls with rate limiting
%   Virtual serial/UART command transmission to motors
%   Visual representation through 3D animation
%   Forward Kinematics check (round-trip calculation)
%   Four graphs including tracking error

% Approach: Inverse Kinematics Directly used
% Inverse Kinematics Pose: Elbow-Up pose only

% ASSUMPTIONS:
%   Elbow-up Inverse Kinematic solution is exclusively used.
%   Links considered as rigid — no flexibility modelled.
%   Joint actuators as ideal — no back-lash, friction, or inertia.
%   Base is fixed rigidly at Cartesian origin (0,0,0).
%   Gravity and dynamics not included (kinematic model only).
%   No encoder quantization effects — perfect feedback.
%   No communication delays between controller and motors.
%   PID plant model is an integrator (idealized motor model).
%   Target points within a 30x30x30mm working envelope.
%   Origin point (0,0,0) represents the starting point,
%   since this point is inside the dead zone (r < |L1-L2|).

clc; clear; close all;

%  PART 1: PARAMETERS OF ROBOT ARM DESIGN  & REACHABILITY PROOF

% theta1 = base yaw (about Z), theta2 = shoulder pitch, theta3 = elbow pitch.
% Two physical links: L1 (base to elbow), L2 (elbow to end-effector).
% Base sits at the world origin.

L1 = 30;   % Link 1 length in mm
L2 = 25;   % Link 2 length in mm

gridSize = 30;   % workspace cube side in mm


%  inner radius = |L1 - L2|  (arm is fully folded)
%  outer radius = L1 + L2    (arm is fully extended)
innerR = abs(L1 - L2);   %  5 mm
outerR = L1 + L2;        % 55 mm

diag3D = sqrt(3) * gridSize;   % = sqrt(30^2+30^2+30^2) ~ 52 mm

fprintf('=== Robot Arm Design ===\n');
fprintf('Link 1 (L1)          = %d mm\n', L1);
fprintf('Link 2 (L2)          = %d mm\n', L2);
fprintf('Max reach (L1+L2)    = %d mm\n', outerR);
fprintf('Min reach |L1-L2|    = %d mm\n', innerR);
fprintf('Workspace diagonal   = %.2f mm\n', diag3D);
fprintf('Arm reaches diagonal : %s\n\n', string(outerR >= diag3D));

% Full chessboard reachability sweep 
% Grid points are every 5 mm from 0..30 on each axis (covers all intersections of the 30x30 mm chessboard squares in 3D).
fprintf('--- Chessboard Grid Reachability Sweep (step=5mm) ---\n');
allReachable = true;
deadZoneCount = 0;
maxDist = 0;
worstPoint = [0 0 0];

for gx = 0:5:gridSize
    for gy = 0:5:gridSize
        for gz = 0:5:gridSize
            d = sqrt(gx^2 + gy^2 + gz^2);
            if d > maxDist
                maxDist = d;
                worstPoint = [gx gy gz];
            end
            if d > outerR
                allReachable = false;
            elseif d > 0 && d < innerR
                deadZoneCount = deadZoneCount + 1;
            end
        end
    end
end

fprintf('Furthest grid point  = (%g,%g,%g), dist = %.2f mm\n', ...
    worstPoint(1), worstPoint(2), worstPoint(3), maxDist);
fprintf('Reachable shell      = %.0f mm to %.0f mm\n', innerR, outerR);
fprintf('Non-origin dead-zone points: %d\n', deadZoneCount);
fprintf('All non-origin grid points reachable: %s\n\n', ...
    string(allReachable && deadZoneCount == 0));


%  PART 2: WAYPOINT INPUT MANUALLY
%  To start it, create waypoints.txt with the 4 given points
%  Loads from waypoints.txt if the file exists, OR prompts the user interactively, OR falls back to demo points.

demoPoints = [  0,  0,  0;
               10, 10, 25;
               25,  5, 25;
               20, 25, 25;
                5, 20, 25 ];

wpFile = 'waypoints.txt';

if exist(wpFile, 'file') == 2
    % Option A: load from file
    loaded = load(wpFile);
    if size(loaded,2) == 3 && size(loaded,1) >= 2
        points = loaded;
        fprintf('Loaded %d waypoints from %s\n', size(points,1), wpFile);
    else
        points = demoPoints;
        fprintf('%s wrong format — using demo waypoints.\n', wpFile);
    end
else
    % Option B: Manual input
    fprintf('=== Waypoint Input ===\n');
    fprintf('No waypoints.txt found.\n');
    fprintf('Enter 4 target waypoints (x y z in mm, within 0-30 each).\n');
    fprintf('Press ENTER on any line to use the built-in demo points.\n\n');

    useDemo = false;
    waypoints = zeros(4, 3);
    for i = 1:4
        def = demoPoints(i+1, :);
        prompt = sprintf('  P%d (x y z) [demo: %g %g %g]: ', i, def(1), def(2), def(3));
        raw = input(prompt, 's');
        if isempty(strtrim(raw))
            useDemo = true;
            break;
        end
        vals = str2num(raw); 
        if numel(vals) == 3
            waypoints(i,:) = vals;
        else
            fprintf('  Invalid — using demo points.\n');
            useDemo = true;
            break;
        end
    end

    if useDemo
        points = demoPoints;
        fprintf('Using demo waypoints.\n');
    else
        points = [0 0 0; waypoints];
    end
end

numPoints = size(points, 1);

fprintf('\n=== Confirmed Waypoints ===\n');
for i = 1:numPoints
    d = norm(points(i,:));
    inWS = all(points(i,:) >= 0) && all(points(i,:) <= gridSize);
    if inWS
        status = 'OK';
    else
        status = 'WARNING: outside workspace!';
    end
    fprintf('P%d = (%5.1f, %5.1f, %5.1f) mm  dist=%.2f mm  %s\n', ...
        i-1, points(i,1), points(i,2), points(i,3), d, status);
end
fprintf('\n');


%  PART 3: INVERSE KINEMATICS
%  Direct geometric Inverse Kinematics used.
%  Elbow-Up (positive sqrt for theta3).

%  Equations:
%    theta1 = atan2(y, x)
%    r      = sqrt(x^2 + y^2)           [horizontal reach]
%    D      = (r^2+z^2-L1^2-L2^2)/(2*L1*L2)   [law of cosines]
%    theta3 = atan2(+sqrt(1-D^2), D)    [elbow-up + root]
%    theta2 = atan2(z,r) - atan2(L2*sin(t3), L1+L2*cos(t3))
% ------------------------------------------------------------------------

fprintf('=== Inverse Kinematics Results ===\n');
fprintf('Method: Direct Geometric IK  |  Config: Elbow-Up\n\n');
angles   = zeros(numPoints, 3);
wpValid  = false(numPoints, 1);

for i = 1:numPoints
    [th1, th2, th3, valid] = inverseKinematics( ...
        points(i,1), points(i,2), points(i,3), L1, L2);
    angles(i,:) = [th1, th2, th3];
    wpValid(i)  = valid;
    if valid
        fprintf('P%d -> theta1=%7.2f deg  theta2=%7.2f deg  theta3=%7.2f deg\n', ...
            i-1, th1, th2, th3);
    else
        fprintf('P%d -> *** UNREACHABLE ***\n', i-1);
    end
end
fprintf('\n');


%  PART 4: TRAJECTORY PLANNING
%  Straight-line Cartesian interpolation:
%    P(t) = P_start + t*(P_end - P_start),  t in [0,1], N steps
%  The origin (home pose) lies in the arm dead zone and is excluded from the tracked path. Only reachable waypoints are connected.

% Collect only reachable, non-dead-zone waypoints for path
trajPoints = [];
for i = 1:numPoints
    d = norm(points(i,:));
    if d >= innerR && d <= outerR
        trajPoints(end+1, :) = points(i,:); 
    end
end
numTraj = size(trajPoints, 1);

fprintf('Path tracing: %d of %d waypoints (origin/home excluded)\n', ...
    numTraj, numPoints);

if numTraj < 2
    error('Fewer than 2 reachable waypoints — nothing to trace. Check coordinates.');
end

N = 50;   % interpolation steps per segment
fullTraj_pos    = [];
fullTraj_angles = [];

for seg = 1:(numTraj - 1)
    P_start   = trajPoints(seg,   :);
    P_end     = trajPoints(seg+1, :);
    firstStep = 0;
    if seg > 1, firstStep = 1; end   % avoid duplicate join point

    for step = firstStep:N
        t = step / N;
        x = P_start(1) + t*(P_end(1) - P_start(1));
        y = P_start(2) + t*(P_end(2) - P_start(2));
        z = P_start(3) + t*(P_end(3) - P_start(3));

        fullTraj_pos(end+1, :) = [x, y, z]; 

        [a1, a2, a3, ~] = inverseKinematics(x, y, z, L1, L2);
        fullTraj_angles(end+1, :) = [a1, a2, a3]; 
    end
end

fprintf('Trajectory: %d segments, %d interpolated points\n\n', ...
    numTraj-1, size(fullTraj_pos,1));


%  Forward Kinematic Verification
%  Forward Kinematic round-trip confirms equation correctness.
%  Expected error < 0.001 mm. Large error = Inverse Kinematic bug.


fprintf('=== FK Verification (IK -> FK round-trip) ===\n');
maxErr = 0;
for i = 1:numPoints
    isOrigin = all(abs(points(i,:)) < 1e-6);
    if isOrigin
        fprintf('P%d: home pose (origin — not tracked, skipped)\n', i-1);
        continue;
    end
    if ~wpValid(i)
        fprintf('P%d: unreachable — skipped\n', i-1);
        continue;
    end
    [~, ~, ee_fk] = forwardKinematics(angles(i,1), angles(i,2), angles(i,3), L1, L2);
    err = norm(ee_fk - points(i,:));
    maxErr = max(maxErr, err);
    fprintf('P%d: target=(%6.2f,%6.2f,%6.2f)  FK=(%6.2f,%6.2f,%6.2f)  err=%.4f mm\n', ...
        i-1, points(i,1),points(i,2),points(i,3), ...
        ee_fk(1),ee_fk(2),ee_fk(3), err);
end
fprintf('Max round-trip error: %.4f mm  (should be < 0.001)\n\n', maxErr);


%  PART 5: PID CONTROL LOOP

%  Each joint driven by an independent PID controller.

%  PID formula:
%    e(t)   = theta_desired(t) - theta_actual(t)
%    P      = Kp * e(t)
%    I      = Ki * sum(e * dt)            [accumulated integral]
%    D      = Kd * (e(t) - e(t-1)) / dt  [finite-difference derivative]
%    u(t)   = P + I + D                  [motor rate command, deg/s]
%    theta_actual(t+1) = theta_actual(t) + u(t)*dt

%  Rate limiter: clamps u to +/- rateLimit deg/s to prevent unrealistically large commands.

%  Communication: at every step the joint command is formatted into a serial packet <J1:v,J2:v,J3:v> and passed to sendMotorCommand().
%  On real hardware this packet would be written to a UART/serial port.

Kp = 2.0;
Ki = 0.05;
Kd = 0.8;
dt = 0.02;          % 50 Hz control loop
rateLimit = 400;    % deg/s clamp

fprintf('=== PID Control Simulation ===\n');
fprintf('Kp=%.2f  Ki=%.4f  Kd=%.2f  dt=%.3f s  rateLimit=%d deg/s\n\n', ...
    Kp, Ki, Kd, dt, rateLimit);

numSteps      = size(fullTraj_angles, 1);
actual_angles = zeros(numSteps, 3);
actual_angles(1,:) = fullTraj_angles(1,:);

integral_err   = [0 0 0];
prev_err       = [0 0 0];
control_signal = zeros(numSteps, 3);

fprintf('%-6s %-8s %-10s %-10s %-10s  %s\n', ...
    'Step','t (s)','u_J1(d/s)','u_J2(d/s)','u_J3(d/s)','Serial packet (every 20 steps)');
fprintf('%s\n', repmat('-', 1, 75));

for t = 2:numSteps
    desired = fullTraj_angles(t, :);
    actual  = actual_angles(t-1, :);

    err          = desired - actual;
    integral_err = integral_err + err * dt;
    derivative   = (err - prev_err) / dt;

    u = Kp*err + Ki*integral_err + Kd*derivative;
    u = max(min(u, rateLimit), -rateLimit);   % rate limit

    actual_angles(t,:)  = actual + u * dt;
    control_signal(t,:) = u;
    prev_err = err;

    % Simulated serial/UART motor command transmission
    % Packet format: <J1:angle,J2:angle,J3:angle>
    % On real hardware: fprintf(serialPort, packet)
    if mod(t, 20) == 0
        packet = sendMotorCommand(t, actual_angles(t,:), false);
        fprintf('%-6d %-8.3f %-10.2f %-10.2f %-10.2f  %s\n', ...
            t, (t-1)*dt, u(1), u(2), u(3), packet);
    end
end

trackErr = abs(fullTraj_angles - actual_angles);
fprintf('\nPID tracking error  max=%.3f deg  mean=%.3f deg\n\n', ...
    max(trackErr(:)), mean(trackErr(:)));


%  PART 6: PLOTS


timeVec = (0:numSteps-1) * dt;

% Plot 1: 3D Trajectory
figure('Name','3D Trajectory','NumberTitle','off');
plot3(fullTraj_pos(:,1), fullTraj_pos(:,2), fullTraj_pos(:,3), ...
    'b-', 'LineWidth', 2); hold on;
scatter3(points(:,1), points(:,2), points(:,3), 120, 'r', 'filled');
for i = 1:numPoints
    text(points(i,1)+0.5, points(i,2)+0.5, points(i,3)+0.5, ...
        sprintf('P%d', i-1), 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'red');
end
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('End-Effector Trajectory (Straight-Line Interpolation)');
legend('Trajectory','Waypoints','Location','best');
grid on; axis equal;
xlim([0 gridSize]); ylim([0 gridSize]); zlim([0 gridSize]);
view([45, 25]);

% Plot 2: Joint Angles (Desired vs Actual)
figure('Name','Joint Angles','NumberTitle','off');
jointNames = {'Joint 1 – Base (\theta_1)', ...
              'Joint 2 – Shoulder (\theta_2)', ...
              'Joint 3 – Elbow (\theta_3)'};
for j = 1:3
    subplot(3,1,j);
    plot(timeVec, fullTraj_angles(:,j), 'b--', 'LineWidth', 1.5); hold on;
    plot(timeVec, actual_angles(:,j),   'r-',  'LineWidth', 1.2);
    ylabel('Angle (deg)'); title(jointNames{j});
    legend('Desired (IK)','Actual (PID)','Location','best'); grid on;
end
xlabel('Time (s)');
sgtitle('PID Joint Angle Tracking');

% Plot 3: PID Motor Command Signals 
figure('Name','PID Motor Commands','NumberTitle','off');
plot(timeVec, control_signal(:,1), 'r-', 'LineWidth', 1.2); hold on;
plot(timeVec, control_signal(:,2), 'g-', 'LineWidth', 1.2);
plot(timeVec, control_signal(:,3), 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Command u (deg/s)');
title('Simulated Motor Command Signals (Serial/UART output)');
legend('Joint 1','Joint 2','Joint 3','Location','best'); grid on;

% Plot 4: PID Tracking Error per Joint 
figure('Name','Tracking Error','NumberTitle','off');
for j = 1:3
    subplot(3,1,j);
    plot(timeVec, trackErr(:,j), 'm-', 'LineWidth', 1.2);
    ylabel('Error (deg)'); title(['Tracking Error — ' jointNames{j}]); grid on;
end
xlabel('Time (s)');
sgtitle('PID Tracking Error (|Desired – Actual|)');


%  PART 7: 3D ANIMATION
%  Shows 2 physical links and 3 joints labelled explicitly.
%  Ideal Inverse Kinematic angles used so end-effector coincides with trajectory.


figure('Name','Robot Arm Animation','NumberTitle','off');
set(gcf, 'Color', 'white', 'Position', [100 100 720 600]);
fprintf('Playing animation... (close window to stop)\n');

for i = 1:2:numSteps
    if ~ishandle(gcf), break; end
    cla;

    th1 = fullTraj_angles(i,1);
    th2 = fullTraj_angles(i,2);
    th3 = fullTraj_angles(i,3);

    [j0, j1, ee] = forwardKinematics(th1, th2, th3, L1, L2);

    hold on;

    plot3(fullTraj_pos(:,1), fullTraj_pos(:,2), fullTraj_pos(:,3), ...
        'Color', [0.8 0.8 0.8], 'LineWidth', 1, 'LineStyle', '--');

    plot3(fullTraj_pos(1:i,1), fullTraj_pos(1:i,2), fullTraj_pos(1:i,3), ...
        'b--', 'LineWidth', 1.5);

    % Link 1: base joint to elbow joint
    plot3([j0(1) j1(1)], [j0(2) j1(2)], [j0(3) j1(3)], ...
        'b-', 'LineWidth', 5);

    % Link 2: elbow joint to end-effector
    plot3([j1(1) ee(1)], [j1(2) ee(2)], [j1(3) ee(3)], ...
        'g-', 'LineWidth', 5);

    % Joint 1 : Base (black)
    scatter3(j0(1), j0(2), j0(3), 180, 'k', 'filled');

    % Joint 2 : Elbow (blue)
    scatter3(j1(1), j1(2), j1(3), 150, [0.2 0.2 0.8], 'filled');

    % Joint 3 : End-Effector (red)
    scatter3(ee(1), ee(2), ee(3), 160, 'r', 'filled');

    % Waypoints
    scatter3(points(:,1), points(:,2), points(:,3), 100, 'm', 'filled');

    % Waypoint labels
    for p = 1:numPoints
        text(points(p,1)+0.8, points(p,2)+0.8, points(p,3)+1.0, ...
            sprintf('P%d', p-1), 'FontSize', 10, 'FontWeight', 'bold', ...
            'Color', [0.7 0 0.7]);
    end

    % Current joint angles in title
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title(sprintf('3DOF Robot Arm  |  t=%.2fs  \\theta_1=%.1f°  \\theta_2=%.1f°  \\theta_3=%.1f°', ...
        (i-1)*dt, th1, th2, th3), 'FontWeight', 'bold');
    legend('Full path','Traced path', ...
        'Link 1 (L1=30mm)','Link 2 (L2=25mm)', ...
        'Joint 1 – Base','Joint 2 – Elbow','End-Effector','Waypoints', ...
        'Location','northeast','FontSize', 8);

    xlim([0 gridSize]); ylim([0 gridSize]); zlim([0 gridSize]);
    grid on; view([45, 25]);
    drawnow; pause(0.01);
end

fprintf('\nSimulation complete.\n');

%  LOCAL FUNCTIONS

function [th1, th2, th3, valid] = inverseKinematics(x, y, z, L1, L2)
% Direct closed-form geometric Inverse Kinematic for base-yaw + shoulder + elbow arm.
% Uses elbow-up configuration (positive sqrt for theta3).
% Returns joint angles in degrees.

    valid = true;

    % Home pose: all joints at zero
    if x == 0 && y == 0 && z == 0
        th1 = 0; th2 = 0; th3 = 0;
        return;
    end

    % Joint 1: base yaw — rotate arm to face target in XY plane
    th1 = atan2d(y, x);

    % Horizontal distance from base axis to target
    r = sqrt(x^2 + y^2);

    % Law of cosines — reachability parameter D
    D = (r^2 + z^2 - L1^2 - L2^2) / (2 * L1 * L2);

    % Guard floating-point edge cases at workspace boundary
    if abs(D) > 1
        if abs(D) < 1.0001
            D = sign(D);   % clamp numerical overshoot at boundary
        else
            warning('IK: point (%.2f,%.2f,%.2f) unreachable. D=%.4f', x, y, z, D);
            th1 = 0; th2 = 0; th3 = 0;
            valid = false;
            return;
        end
    end

    % Joint 3: elbow — elbow-up = positive square root
    th3 = atan2d(sqrt(1 - D^2), D);

    % Joint 2: shoulder
    th2 = atan2d(z, r) - atan2d(L2 * sind(th3), L1 + L2 * cosd(th3));
end


function [j0, j1, ee] = forwardKinematics(th1, th2, th3, L1, L2)
% Forward kinematics for base-yaw + shoulder + elbow arm.
% Returns positions of:
%   j0 = Base joint  (world origin, fixed)
%   j1 = Elbow joint (end of Link 1)
%   ee = End-Effector (end of Link 2)

    j0 = [0, 0, 0];   % base fixed at world origin

    % Elbow joint : end of Link 1
    j1(1) = L1 * cosd(th1) * cosd(th2);
    j1(2) = L1 * sind(th1) * cosd(th2);
    j1(3) = L1 * sind(th2);

    % End-effector : end of Link 2
    th_tot = th2 + th3;
    ee(1) = j1(1) + L2 * cosd(th1) * cosd(th_tot);
    ee(2) = j1(2) + L2 * sind(th1) * cosd(th_tot);
    ee(3) = j1(3) + L2 * sind(th_tot);
end


function packet = sendMotorCommand(stepIdx, jointAngles, verbose)
% Simulates sending joint angle commands to motor controllers over serial.
% Packet format: <J1:angle,J2:angle,J3:angle>

% On real hardware this would be:
%   fprintf(serialPort, packet);   
% UART/Serial transmission or equivalent CAN/I2C write.

% Here we build and return the packet string so the communication step is represented in the simulation output.

    packet = sprintf('<J1:%.2f,J2:%.2f,J3:%.2f>', ...
        jointAngles(1), jointAngles(2), jointAngles(3));
    if verbose
        fprintf('step %d TX: %s\n', stepIdx, packet);
    end
end