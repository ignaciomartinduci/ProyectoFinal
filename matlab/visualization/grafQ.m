function [] = grafQ(q_traj, qd_traj, qdd_traj)

cfg = config;
dt = cfg.traj_dt;

t = 1:size(q_traj,1);
t = t*dt;

step = 10;


%% ===== POSICION =====
figure;
hold on
subplot(6,1,1)
plot(t, q_traj(:,1)); ylabel('q1 [rad]'); grid minor;
subplot(6,1,2)
plot(t, q_traj(:,2)); ylabel('q2 [rad]'); grid minor;
subplot(6,1,3)
plot(t, q_traj(:,3)); ylabel('q3 [rad]'); grid minor;
subplot(6,1,4)
plot(t, q_traj(:,4)); ylabel('q4 [rad]'); grid minor;
subplot(6,1,5)
plot(t, q_traj(:,5)); ylabel('q5 [rad]'); grid minor;
subplot(6,1,6)
plot(t, q_traj(:,6)); ylabel('q6 [rad]'); xlabel('Tiempo'); grid minor;
sgtitle("q_{traj}")
hold off

%% ===== VELOCIDAD =====
figure;
hold on
subplot(6,1,1)
plot(t, qd_traj(:,1)); ylabel('qd1 [rad/s]'); grid minor;
subplot(6,1,2)
plot(t, qd_traj(:,2)); ylabel('qd2 [rad/s]'); grid minor;
subplot(6,1,3)
plot(t, qd_traj(:,3)); ylabel('qd3 [rad/s]'); grid minor;
subplot(6,1,4)
plot(t, qd_traj(:,4)); ylabel('qd4 [rad/s]'); grid minor;
subplot(6,1,5)
plot(t, qd_traj(:,5)); ylabel('qd5 [rad/s]'); grid minor;
subplot(6,1,6)
plot(t, qd_traj(:,6)); ylabel('qd6 [rad/s]'); xlabel('Tiempo'); grid minor;
sgtitle("qd_{traj}")
hold off

%% ===== ACELERACION =====
figure;
hold on
subplot(6,1,1)
plot(t, qdd_traj(:,1)); ylabel('qdd1 [rad/s^2]'); grid minor;
subplot(6,1,2)
plot(t, qdd_traj(:,2)); ylabel('qdd2 [rad/s^2]'); grid minor;
subplot(6,1,3)
plot(t, qdd_traj(:,3)); ylabel('qdd3 [rad/s^2]'); grid minor;
subplot(6,1,4)
plot(t, qdd_traj(:,4)); ylabel('qdd4 [rad/s^2]'); grid minor;
subplot(6,1,5)
plot(t, qdd_traj(:,5)); ylabel('qdd5 [rad/s^2]'); grid minor;
subplot(6,1,6)
plot(t, qdd_traj(:,6)); ylabel('qdd6 [rad/s^2]'); xlabel('Tiempo'); grid minor;
sgtitle("qdd_{traj}")
hold off

end