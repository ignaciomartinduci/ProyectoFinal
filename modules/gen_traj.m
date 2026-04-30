function [q_traj, dbg] = gen_traj(R, cpoints_input, q0, qdmax, qddmax, verbose, vmax, amax, wmax, alphamax)

%% Inicialización

cfg = config();
dt  = cfg.traj_dt;

q_traj = [];
dbg.q_before = [];
dbg.q_after  = [];

%% Desglose de argumentos

cpoints = cpoints_input(:,1:6);
modes   = cpoints_input(:,7);

%% Interpolaciones

t_acc   = max(qdmax ./ qddmax);
idx_end = -1;
first   = "FORCE";

for i = 1:length(cpoints(:,1))

    %% Modo articular (0)
    if modes(i) == 0 && i > idx_end

        idx_end = i;
        if i < length(cpoints(:,1))
            for j = i+1:length(cpoints(:,1))
                if modes(j) == 1; break; end
                if modes(j) == 0; idx_end = idx_end + 1; end
            end
        end

        % Construcción de waypoints articulares
        WP       = zeros(idx_end - i + 2, 6);
        WP(1,:)  = get_q_ref(q_traj, q0);
        q_previo = WP(1,:);

        for j = i:idx_end
            [~, q_mejor] = inv_kinematics(cpoints(j,1), cpoints(j,2), cpoints(j,3), ...
                                           cpoints(j,4), cpoints(j,5), cpoints(j,6), ...
                                           q_previo, verbose, R, first);
            first    = "NONE";
            q_previo = q_mejor;
            WP(j - i + 2,:) = q_mejor;
        end

        % Trayectoria inicial
        q_before = mstraj(WP(2:end,:), qdmax, [], WP(1,:), dt, t_acc');

        % Escala para cumplir límites en ambos espacios
        scale = compute_scale(R, q_before, qdmax, qddmax, vmax, amax, wmax, alphamax, dt);

        % qdmax/k alarga la duración de segmentos; t_acc*k alarga las rampas
        % -> velocidad escala 1/k, aceleración escala 1/k² (correcto)
        q_after = mstraj(WP(2:end,:), qdmax / scale, [], WP(1,:), dt, (t_acc * scale)');

        q_traj       = [q_traj;       q_after];
        dbg.q_before = [dbg.q_before; q_before];
        dbg.q_after  = [dbg.q_after;  q_after];

    %% Modo cartesiano (1)
    elseif modes(i) == 1

        if i == 1
            T_q0 = R.fkine(q0);
            T_0  = [T_q0.n, T_q0.o, T_q0.a, T_q0.t; 0 0 0 1];
        else
            T_0 = transl(cpoints(i-1,1), cpoints(i-1,2), cpoints(i-1,3)) * ...
                  rpy2tr(cpoints(i-1,4), cpoints(i-1,5), cpoints(i-1,6));
        end

        T_1 = transl(cpoints(i,1), cpoints(i,2), cpoints(i,3)) * ...
              rpy2tr(cpoints(i,4), cpoints(i,5), cpoints(i,6));

        tf = calculate_tf(T_0, T_1, vmax, amax, wmax, alphamax);

        if tf > 0

            q_ref = get_q_ref(q_traj, q0);

            % Trayectoria cartesiana inicial -> articular
            T_temp   = make_ctraj(T_0, T_1, tf, dt);
            q_before = ctraj_to_jtraj(R, T_temp, q_ref, verbose, first);
            first    = "NONE";

            % Escala y regeneración si hay violaciones
            scale = compute_scale(R, q_before, qdmax, qddmax, vmax, amax, wmax, alphamax, dt);

            if scale > 1
                tf       = tf * scale;
                T_temp   = make_ctraj(T_0, T_1, tf, dt);
                q_after  = ctraj_to_jtraj(R, T_temp, q_ref, verbose, "NONE");
            else
                q_after  = q_before;
            end

            q_traj       = [q_traj;       q_after];
            dbg.q_before = [dbg.q_before; q_before];
            dbg.q_after  = [dbg.q_after;  q_after];

        end

    end

end

end

%% -- Helpers ------------------------------------------------------------------

function q_ref = get_q_ref(q_traj, q0)
    if isempty(q_traj); q_ref = q0; else; q_ref = q_traj(end,:); end
end

function T_temp = make_ctraj(T_0, T_1, tf, dt)
    t      = 0:dt:tf;
    tau    = t / tf;
    s      = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    T_temp = ctraj(T_0, T_1, s);
end

function qtraj = ctraj_to_jtraj(R, T_temp, q_ref, verbose, first)
    N        = size(T_temp, 3);
    qtraj    = zeros(N, 6);
    q_previo = q_ref;
    for k = 1:N
        rpy = tr2rpy(T_temp(:,:,k), 'zyx');
        [~, q_mejor] = inv_kinematics(T_temp(1,4,k), T_temp(2,4,k), T_temp(3,4,k), ...
                                       rpy(1), rpy(2), rpy(3), q_previo, verbose, R, first);
        first      = "NONE";
        q_previo   = q_mejor;
        qtraj(k,:) = q_mejor;
    end
end

%% -- Cómputo de escala requerida ---------------------------------------------

function scale = compute_scale(R, qtraj, qdmax, qddmax, vmax, amax, wmax, alphamax, dt)
% Retorna k >= 1 tal que la trayectoria re-escalada cumple todos los limites.
% Al estirar el tiempo por k: velocidades escalan 1/k, aceleraciones 1/k^2.

N = size(qtraj, 1);

qdtraj  = zeros(N, 6);
qddtraj = zeros(N, 6);
for j = 1:6
    qdtraj(:,j)  = gradient(qtraj(:,j),  dt);
    qddtraj(:,j) = gradient(qdtraj(:,j), dt);
end

% Espacio articular
k_qd  = max(max(abs(qdtraj),  [], 1) ./ qdmax);
k_qdd = sqrt(max(max(abs(qddtraj), [], 1) ./ qddmax));

% Espacio cartesiano: velocidad y aceleración reales via Jacobiano
v_lin = zeros(N,1); v_ang = zeros(N,1);
a_lin = zeros(N,1); a_ang = zeros(N,1);

for i = 1:N
    J  = R.jacob0(qtraj(i,:));
    cv = J * qdtraj(i,:)';
    ca = J * qddtraj(i,:)';
    v_lin(i) = norm(cv(1:3));
    v_ang(i) = norm(cv(4:6));
    a_lin(i) = norm(ca(1:3));
    a_ang(i) = norm(ca(4:6));
end

k_v     = max(v_lin) / vmax;
k_w     = max(v_ang) / wmax;
k_a     = sqrt(max(a_lin) / amax);
k_alpha = sqrt(max(a_ang) / alphamax);

scale = max([1, k_qd, k_qdd, k_v, k_w, k_a, k_alpha]);
end

%% -- Tiempo mínimo para trayectoria cartesiana --------------------------------

function tf = calculate_tf(T_0, T_1, vmax, amax, wmax, alphamax)

cfg = config();
d   = norm(T_1(1:3,4) - T_0(1:3,4));

if d < 1e-4
    tf = -1;
    return
end

[theta, ~] = tr2angvec(T_0 \ T_1);

tf_v     =      (cfg.s_dot_max  * d)     / vmax;
tf_a     = sqrt((cfg.s_ddot_max * d)     / amax);
tf_w     =      (cfg.s_dot_max  * theta) / wmax;
tf_alpha = sqrt((cfg.s_ddot_max * theta) / alphamax);

tf = max([tf_v, tf_a, tf_w, tf_alpha]);
end
