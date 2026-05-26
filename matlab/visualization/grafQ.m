function [] = grafQ(R, q_traj, qdmax, qddmax)

cfg = config;
dt  = cfg.traj_dt;
N   = size(q_traj, 1);
t   = (0:N-1)' * dt;

% Derivadas numéricas
qd_traj  = zeros(N, 6);
qdd_traj = zeros(N, 6);
for j = 1:6
    qd_traj(:,j)  = gradient(q_traj(:,j),    dt);
    qdd_traj(:,j) = gradient(qd_traj(:,j), dt);
end

labels = {'q1','q2','q3','q4','q5','q6'};

%% ===== FIGURA 1: Posiciones articulares + límites =====

figure;
sgtitle('Posiciones articulares');
for j = 1:6
    subplot(6,1,j);
    hold on;
    plot(t, q_traj(:,j), 'b');
    yline(R.qlim(j,1), 'r--');
    yline(R.qlim(j,2), 'r--');
    ylabel([labels{j} ' [rad]']);
    grid minor; xlim([t(1) t(end)]);
    if j == 6, xlabel('t [s]'); end
    hold off;
end

%% ===== FIGURA 4: Velocidades y aceleraciones articulares + límites =====

figure;
sgtitle('Velocidades y aceleraciones articulares');
for j = 1:6

    subplot(6,2, 2*j-1);
    hold on;
    plot(t, qd_traj(:,j), 'b');
    yline( qdmax(j), 'r--');
    yline(-qdmax(j), 'r--');
    ylabel(['$\dot{q}_' num2str(j) '$ [rad/s]'], 'Interpreter', 'latex');
    grid minor; xlim([t(1) t(end)]);
    if j == 6, xlabel('t [s]'); end
    hold off;

    subplot(6,2, 2*j);
    hold on;
    plot(t, qdd_traj(:,j), 'b');
    yline( qddmax(j), 'r--');
    yline(-qddmax(j), 'r--');
    ylabel(['$\ddot{q}_' num2str(j) '$ [rad/s$^2$]'], 'Interpreter', 'latex');
    grid minor; xlim([t(1) t(end)]);
    if j == 6, xlabel('t [s]'); end
    hold off;

end

end
