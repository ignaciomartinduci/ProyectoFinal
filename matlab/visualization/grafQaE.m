function [] = grafQaE(R, q_traj, vmax, amax, wmax, alphamax)

cfg = config();
dt  = cfg.traj_dt;
N   = size(q_traj, 1);
t   = (0:N-1)' * dt;

%% --- CINEMÁTICA DIRECTA ---

x = zeros(N,1); y = zeros(N,1); z = zeros(N,1);
roll = zeros(N,1); pitch = zeros(N,1); yaw = zeros(N,1);

for i = 1:N
    Tm     = double(R.fkine(q_traj(i,:)));
    x(i)   = Tm(1,4);
    y(i)   = Tm(2,4);
    z(i)   = Tm(3,4);
    rpy    = tr2rpy(Tm, 'zyx');
    roll(i)  = rpy(1);
    pitch(i) = rpy(2);
    yaw(i)   = rpy(3);
end

roll  = unwrap(roll);
pitch = unwrap(pitch);
yaw   = unwrap(yaw);

%% --- VELOCIDADES Y ACELERACIONES CARTESIANAS ---

qdtraj = zeros(N, 6);
for j = 1:6
    qdtraj(:,j) = gradient(q_traj(:,j), dt);
end

% Velocidad via Jacobiano; aceleración por diferenciación numérica
% (incluye implícitamente el término Jdot*qd)
v_cart = zeros(N, 6);
for i = 1:N
    J = R.jacob0(q_traj(i,:));
    v_cart(i,:) = (J * qdtraj(i,:)')';
end

a_cart = zeros(N, 6);
for j = 1:6
    a_cart(:,j) = gradient(v_cart(:,j), dt);
end

v_lin = vecnorm(v_cart(:,1:3), 2, 2);
v_ang = vecnorm(v_cart(:,4:6), 2, 2);
a_lin = vecnorm(a_cart(:,1:3), 2, 2);
a_ang = vecnorm(a_cart(:,4:6), 2, 2);

%% ===== FIGURA 2: Posición cartesiana EF =====

figure;
subplot(6,1,1); plot(t, x);     grid minor; ylabel('x [m]');       xlim([t(1) t(end)]);
subplot(6,1,2); plot(t, y);     grid minor; ylabel('y [m]');       xlim([t(1) t(end)]);
subplot(6,1,3); plot(t, z);     grid minor; ylabel('z [m]');       xlim([t(1) t(end)]);
subplot(6,1,4); plot(t, roll);  grid minor; ylabel('roll [rad]');  xlim([t(1) t(end)]);
subplot(6,1,5); plot(t, pitch); grid minor; ylabel('pitch [rad]'); xlim([t(1) t(end)]);
subplot(6,1,6); plot(t, yaw);   grid minor; ylabel('yaw [rad]'); xlabel('t [s]'); xlim([t(1) t(end)]);
sgtitle('Posición cartesiana EF');

%% ===== FIGURA 3: Velocidades y aceleraciones EF + límites =====

figure;
sgtitle('Velocidades y aceleraciones EF');

subplot(4,1,1);
hold on;
plot(t, v_lin, 'b', 'DisplayName', '|v|');
yline(vmax, 'r--', 'DisplayName', 'v_{max}');
ylabel('|v| [m/s]'); grid minor; xlim([t(1) t(end)]);
legend('Location','northeast'); hold off;

subplot(4,1,2);
hold on;
plot(t, a_lin, 'b', 'DisplayName', '|a|');
yline(amax, 'r--', 'DisplayName', 'a_{max}');
ylabel('|a| [m/s^2]'); grid minor; xlim([t(1) t(end)]); hold off;

subplot(4,1,3);
hold on;
plot(t, v_ang, 'b', 'DisplayName', '|\omega|');
yline(wmax, 'r--', 'DisplayName', '\omega_{max}');
ylabel('|\omega| [rad/s]'); grid minor; xlim([t(1) t(end)]); hold off;

subplot(4,1,4);
hold on;
plot(t, a_ang, 'b', 'DisplayName', '|\alpha|');
yline(alphamax, 'r--', 'DisplayName', '\alpha_{max}');
ylabel('|\alpha| [rad/s^2]'); xlabel('t [s]'); grid minor; xlim([t(1) t(end)]); hold off;

end
