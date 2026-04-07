function [] = grafQaE(R, q_traj)

    cfg = config();
    dt = cfg.traj_dt;

    N = size(q_traj,1);

    t = (0:N-1)' * dt;

    T = zeros(4,4,N);

    x = zeros(N,1);
    y = zeros(N,1);
    z = zeros(N,1);
    roll  = zeros(N,1);
    pitch = zeros(N,1);
    yaw   = zeros(N,1);
    
    %% --- CINEMÁTICA DIRECTA ---
    for i = 1:N

        T(:,:,i) = R.fkine(q_traj(i,:));

        x(i) = T(1,4,i);
        y(i) = T(2,4,i);
        z(i) = T(3,4,i);

        rpy = tr2rpy(T(:,:,i),'zyx');

        roll(i)  = rpy(1);
        pitch(i) = rpy(2);
        yaw(i)   = rpy(3);

    end

    %% --- DERIVADAS NUMÉRICAS ---

    vx = gradient(x, dt);
    vy = gradient(y, dt);
    vz = gradient(z, dt);

    vroll  = gradient(roll, dt);
    vpitch = gradient(pitch, dt);
    vyaw   = gradient(yaw, dt);

    ax = gradient(vx, dt);
    ay = gradient(vy, dt);
    az = gradient(vz, dt);

    aroll  = gradient(vroll, dt);
    apitch = gradient(vpitch, dt);
    ayaw   = gradient(vyaw, dt);

    %% --- PLOTS ---

    % ===== POSICIÓN =====
    figure;
    subplot(6,1,1); plot(t,x); grid minor; ylabel('x [m]'); xlim([t(1) t(end)]);
    subplot(6,1,2); plot(t,y); grid minor; ylabel('y [m]'); xlim([t(1) t(end)]);
    subplot(6,1,3); plot(t,z); grid minor; ylabel('z [m]'); xlim([t(1) t(end)]);
    subplot(6,1,4); plot(t,roll); grid minor; ylabel('roll [rad]'); xlim([t(1) t(end)]);
    subplot(6,1,5); plot(t,pitch); grid minor; ylabel('pitch [rad]'); xlim([t(1) t(end)]);
    subplot(6,1,6); plot(t,yaw); grid minor; ylabel('yaw [rad]'); xlabel('t [s]'); xlim([t(1) t(end)]);
    sgtitle('Posición cartesiana');

    % ===== VELOCIDAD =====
    figure;
    subplot(6,1,1); plot(t,vx); grid minor; ylabel('vx [m/s]'); xlim([t(1) t(end)]);
    subplot(6,1,2); plot(t,vy); grid minor; ylabel('vy [m/s]'); xlim([t(1) t(end)]);
    subplot(6,1,3); plot(t,vz); grid minor; ylabel('vz [m/s]'); xlim([t(1) t(end)]);
    subplot(6,1,4); plot(t,vroll); grid minor; ylabel('vroll [rad/s]'); xlim([t(1) t(end)]);
    subplot(6,1,5); plot(t,vpitch); grid minor; ylabel('vpitch [rad/s]'); xlim([t(1) t(end)]);
    subplot(6,1,6); plot(t,vyaw); grid minor; ylabel('vyaw [rad/s]'); xlabel('t [s]'); xlim([t(1) t(end)]);
    sgtitle('Velocidad cartesiana');

    % ===== ACELERACIÓN =====
    figure;
    subplot(6,1,1); plot(t,ax); grid minor; ylabel('ax [m/s^2]'); xlim([t(1) t(end)]);
    subplot(6,1,2); plot(t,ay); grid minor; ylabel('ay [m/s^2]'); xlim([t(1) t(end)]);
    subplot(6,1,3); plot(t,az); grid minor; ylabel('az [m/s^2]'); xlim([t(1) t(end)]);
    subplot(6,1,4); plot(t,aroll); grid minor; ylabel('aroll [rad/s^2]'); xlim([t(1) t(end)]);
    subplot(6,1,5); plot(t,apitch); grid minor; ylabel('apitch [rad/s^2]'); xlim([t(1) t(end)]);
    subplot(6,1,6); plot(t,ayaw); grid minor; ylabel('ayaw [rad/s^2]'); xlabel('t [s]'); xlim([t(1) t(end)]);
    sgtitle('Aceleración cartesiana');

end