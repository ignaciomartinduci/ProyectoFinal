function grafScaling(R, dbg, qdmax, vmax, amax, wmax, alphamax)

cfg = config();
dt  = cfg.traj_dt;

q_before = dbg.q_before;
q_after  = dbg.q_after;

if isempty(q_before)
    disp('grafScaling: sin datos de segmentos modo 0, nada que graficar.');
    return;
end

N_b = size(q_before, 1);
N_a = size(q_after,  1);

t_b = (0:N_b-1)' * dt;
t_a = (0:N_a-1)' * dt;

%% --- Velocidades articulares ---

qd_b = zeros(N_b, 6);
qd_a = zeros(N_a, 6);
for j = 1:6
    qd_b(:,j) = gradient(q_before(:,j), dt);
    qd_a(:,j) = gradient(q_after(:,j),  dt);
end

%% --- Velocidades del EF via Jacobiano ---

qdd_b = zeros(N_b, 6);
qdd_a = zeros(N_a, 6);
for j = 1:6
    qdd_b(:,j) = gradient(qd_b(:,j), dt);
    qdd_a(:,j) = gradient(qd_a(:,j), dt);
end

v_b     = zeros(N_b,1);  w_b     = zeros(N_b,1);
a_b     = zeros(N_b,1);  alpha_b = zeros(N_b,1);

for i = 1:N_b
    J       = R.jacob0(q_before(i,:));
    ee_vel  = J * qd_b(i,:)';
    ee_acc  = J * qdd_b(i,:)';     % aproximacion: ignora Jdot*qd
    v_b(i)     = norm(ee_vel(1:3));
    w_b(i)     = norm(ee_vel(4:6));
    a_b(i)     = norm(ee_acc(1:3));
    alpha_b(i) = norm(ee_acc(4:6));
end

v_a     = zeros(N_a,1);  w_a     = zeros(N_a,1);
a_a     = zeros(N_a,1);  alpha_a = zeros(N_a,1);

for i = 1:N_a
    J       = R.jacob0(q_after(i,:));
    ee_vel  = J * qd_a(i,:)';
    ee_acc  = J * qdd_a(i,:)';
    v_a(i)     = norm(ee_vel(1:3));
    w_a(i)     = norm(ee_vel(4:6));
    a_a(i)     = norm(ee_acc(1:3));
    alpha_a(i) = norm(ee_acc(4:6));
end

%% --- FIGURA 1: Velocidades articulares ---

labels = {'q1','q2','q3','q4','q5','q6'};

figure;
sgtitle('Velocidades articulares: efecto del escalado');
for j = 1:6
    subplot(6,1,j);
    hold on;
    plot(t_b, abs(qd_b(:,j)), 'b',   'DisplayName', 'Sin escalar');
    plot(t_a, abs(qd_a(:,j)), 'r',   'DisplayName', 'Escalado');
    yline(qdmax(j),           'k--', 'DisplayName', 'qdmax');
    ylabel([labels{j} ' [rad/s]']);
    if j == 6, xlabel('t [s]'); end
    grid minor;
    xlim([0, max(t_b(end), t_a(end))]);
    if j == 1, legend('Location','northeast'); end
    hold off;
end

%% --- FIGURA 2: Velocidades del efector final ---

figure;
sgtitle('Velocidades EF: efecto del escalado');

subplot(4,1,1);
hold on;
plot(t_b, v_b,    'b',   'DisplayName', 'Sin escalar');
plot(t_a, v_a,    'r',   'DisplayName', 'Escalado');
yline(vmax,       'k--', 'DisplayName', 'vmax');
ylabel('|v| [m/s]'); grid minor; xlim([0, max(t_b(end), t_a(end))]);
legend('Location','northeast'); hold off;

subplot(4,1,2);
hold on;
plot(t_b, a_b,    'b',   'DisplayName', 'Sin escalar');
plot(t_a, a_a,    'r',   'DisplayName', 'Escalado');
yline(amax,       'k--', 'DisplayName', 'amax');
ylabel('|a| [m/s^2]'); grid minor; xlim([0, max(t_b(end), t_a(end))]);
hold off;

subplot(4,1,3);
hold on;
plot(t_b, w_b,    'b',   'DisplayName', 'Sin escalar');
plot(t_a, w_a,    'r',   'DisplayName', 'Escalado');
yline(wmax,       'k--', 'DisplayName', 'wmax');
ylabel('|\omega| [rad/s]'); grid minor; xlim([0, max(t_b(end), t_a(end))]);
hold off;

subplot(4,1,4);
hold on;
plot(t_b, alpha_b, 'b',  'DisplayName', 'Sin escalar');
plot(t_a, alpha_a, 'r',  'DisplayName', 'Escalado');
yline(alphamax,    'k--', 'DisplayName', 'alphamax');
ylabel('|\alpha| [rad/s^2]'); xlabel('t [s]'); grid minor;
xlim([0, max(t_b(end), t_a(end))]);
hold off;

end
