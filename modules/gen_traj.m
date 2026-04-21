function q_traj = gen_traj(R, cpoints_input, q0, qdmax, qddmax, verbose, amax, vmax, wmax, alphamax)

% Descripción:
%
%   Función definida por usuario.
%
%   Genera trayectorias en coordenadas articulares mediante
%   interpolación cartesiana o articular (según se desee).
%
%   Perfoma cálculo de cinemática inversa según corresponda utilizando
%   el script de cálculo algebráico inv_kinematics.m, también definido
%   por el usuario.
%
%   (Pendiente) Provee robustez frente a singularidades buscando
%   trayectorias alternativas sin saltear puntos de trayectoria
%   cartesiana de entrada. (HECHO:) Lo mismo para puntos que no tienen respuesta
%   y así evitar saltos grandes por acumulación de q_previo.
%
% Argumentos:
%
%   cpoints -> arreglo nx6 de puntos cartesianos a interpolar | nx7 si
%              se especifica tipo de interpolación cartesiana/articular
%
%   q0      -> coordenada articular inicial
%

%% Inicialización de variables

cfg = config();
dt = cfg.traj_dt;

q_traj = [];

%% Desglose de argumentos

cpoints = cpoints_input(:,1:6);
modes = (cpoints_input(:,7));

%% Interpolaciones

t_acc = max(qdmax./qddmax);
idx_end = -1;

first = "FORCE";

for i=1:length(cpoints(:,1))

    if modes(i) == 0 && i > idx_end

        idx_end = i;

        if i<length(cpoints(:,1))
            for j=i+1:length(cpoints(:,1))

                if modes(j) == 1
                    break;
                end

                if modes(j) == 0
                    idx_end = idx_end +1;
                end

            end
        end

        WP = zeros(idx_end-i+2, 6);

        if isempty(q_traj)
            WP(1,:) = q0;
        else
            WP(1,:) = q_traj(end,:);
        end

        if ~isempty(q_traj)
            q_previo = q_traj(end,:);
        else
            q_previo = q0;
        end

        for j=i:idx_end

            x = cpoints(j,1);
            y = cpoints(j,2);
            z = cpoints(j,3);
            alpha = cpoints(j,4);
            beta = cpoints(j,5);
            gamma = cpoints(j,6);

            if ~isempty(q_traj)
                [~, q_mejor] = inv_kinematics(x,y,z,alpha,beta,gamma,q_previo,verbose,R, first);
                first = "NONE";
            else
                [~, q_mejor] = inv_kinematics(x,y,z,alpha,beta,gamma,q0,verbose,R, first);
                first = "NONE";

            end

            q_previo = q_mejor;

            WP(j-i+2,:) = q_mejor;

        end

        % El siguiente interpolador articular tiene
        %   - control de velocidad articular mediante argumento qdmax
        %   - control de aceleración articular mediente argument t_acc = qdmax/qddmax
        %
        q_temp = mstraj(WP(2:end,:), qdmax, [], WP(1,:), dt,t_acc');
        scale = scale_velocities_j2c(R, q_temp, qdmax, qddmax, vmax, amax, wmax, alphamax, dt);

        t_acc_scaled = t_acc*scale;

        q_temp = mstraj(WP(2:end,:), qdmax, [], WP(1,:), dt,t_acc_scaled');

        q_traj = [q_traj; q_temp];

    elseif modes(i)==1

        if i == 1

            x_1 = cpoints(1,1);
            y_1 = cpoints(1,2);
            z_1 = cpoints(1,3);
            alpha_1 = cpoints(1,4);
            beta_1 = cpoints(1,5);
            gamma_1 = cpoints(1,6);

            T_q0 = R.fkine(q0);
            T_0 = [T_q0.n, T_q0.o, T_q0.a, T_q0.t; [0 0 0 1]];
            T_1 = transl(x_1,y_1,z_1) * rpy2tr(alpha_1,beta_1,gamma_1);

            tf = calculate_tf(T_0, T_1, vmax, amax, wmax, alphamax);

            if tf > 0

                t = 0:dt:tf;
                tau = t/tf;

                s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

                T_temp = ctraj(T_0, T_1, s);

                scale = scale_velocities_c2j(R,T_temp, qdmax, qddmax, vmax, amax, wmax, alphamax);

                if scale > 1

                    tf = tf * scale;

                end

                t = 0:dt:tf;
                tau = t/tf;

                s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

                T_temp = ctraj(T_0, T_1, s);

            else

                T_temp = T_0;

            end

        else

            x_0 = cpoints(i-1,1);
            y_0 = cpoints(i-1,2);
            z_0 = cpoints(i-1,3);
            alpha_0 = cpoints(i-1,4);
            beta_0 = cpoints(i-1,5);
            gamma_0 = cpoints(i-1,6);


            x_1 = cpoints(i,1);
            y_1 = cpoints(i,2);
            z_1 = cpoints(i,3);
            alpha_1 = cpoints(i,4);
            beta_1 = cpoints(i,5);
            gamma_1 = cpoints(i,6);

            T_0 = transl(x_0,y_0,z_0) * rpy2tr(alpha_0,beta_0,gamma_0);
            T_1 = transl(x_1,y_1,z_1) * rpy2tr(alpha_1,beta_1,gamma_1);

            tf = calculate_tf(T_0, T_1, vmax, amax, wmax, alphamax);

            if tf > 0

                t = 0:dt:tf;
                tau = t/tf;

                s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

                T_temp = ctraj(T_0, T_1, s);

                scale = scale_velocities_c2j(R,T_temp, qdmax, qddmax, vmax, amax, wmax, alphamax);

                if scale > 1

                    tf = tf * scale;

                end

                t = 0:dt:tf;
                tau = t/tf;

                s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

                T_temp = ctraj(T_0, T_1, s);

            else

                T_temp = T_0;

            end

        end

        for k=1:length(T_temp(1,1,:))

            x = T_temp(1,4,k);
            y = T_temp(2,4,k);
            z = T_temp(3,4,k);

            rpy = tr2rpy(T_temp(:,:,k),'zyx');

            alpha = rpy(1);
            beta = rpy(2);
            gamma = rpy(3);


            if ~isempty(q_traj)
                [~, q_mejor] = inv_kinematics(x,y,z,alpha,beta,gamma,q_traj(end,:),verbose,R, first);
                first = "NONE";
            else
                [~, q_mejor] = inv_kinematics(x,y,z,alpha,beta,gamma,q0,verbose,R, first);
                first = "NONE";
            end

            q_traj = [q_traj; q_mejor];

        end

    end



end

end

function scale = scale_velocities_c2j(R, T, qdmax, qddmax, vmax, amax, wmax, alphamax)

N = size(T,3);

scale_qd_all  = zeros(N,6);
scale_qdd_all = zeros(N,6);

q_previo = zeros(1,6);

for i = 1:N

    x_t = T(1,4,i);
    y_t = T(2,4,i);
    z_t = T(3,4,i);

    rpy = tr2rpy(T(:,:,i),'zyx');
    alpha = rpy(1);
    beta  = rpy(2);
    gamma = rpy(3);

    [~, q_i] = inv_kinematics(x_t, y_t, z_t, alpha, beta, gamma, q_previo, false, R);
    q_previo = q_i;


    J = R.jacob0(q_i);
    Jinv = pinv(J);


    for j = 1:6 % Analizo la saturación de movimiento de cada articulación

        Jv_row = Jinv(j,1:3);
        Jw_row = Jinv(j,4:6);

        nv = norm(Jv_row);
        nw = norm(Jw_row);

        scale_qd_all(i,j) = sqrt( (nv * vmax)^2 + (nw * wmax)^2 )*0.5 / qdmax(j);
        scale_qdd_all(i,j) = sqrt( (nv * amax)^2 + (nw * alphamax)^2 )*0.5 / qddmax(j);
    end

end

scale_qd  = max(scale_qd_all,  [], 'all');
scale_qdd = max(scale_qdd_all, [], 'all');

scale = max([1, scale_qd, scale_qdd]);
scale = 1;

end

function tf = calculate_tf(T_0, T_1, vmax, amax, wmax, alphamax)

cfg = config();

p_0 = T_0(1:3,4);
p_1 = T_1(1:3,4);

d = norm(p_1-p_0);

if d < 10^-4

    %disp("d pequeño");

    tf = -1;
    return

end

[theta, ~] = tr2angvec(T_0 \ T_1);

tf_v = (cfg.s_dot_max * d)/(vmax); % Tiempo de velocidad lineal, evito sobrepasar vmax
tf_a = sqrt((cfg.s_ddot_max * d) / amax); % Tiempo de aceleración lineal, evito sobrepasar amax
tf_w = (cfg.s_dot_max * theta) / wmax; % Tiempo de velocidad angular, evito sobrepasar wmax
tf_alpha = sqrt((cfg.s_ddot_max * theta) / alphamax); % Tiempo de aceleración angular, evito sobrepasar alphamax

tf = max([tf_v, tf_a, tf_w, tf_alpha]); % Elección del peor caso, evita sobrepasar ambos

end

function scale = scale_velocities_j2c(R, qtraj, qdmax, qddmax, vmax, amax, wmax, alphamax, dt)

N = length(qtraj(:,1));
fs = 1.25;
scale_v_all  = zeros(N,6);
scale_w_all  = zeros(N,6);
scale_a_all  = zeros(N,6);
scale_alpha_all = zeros(N,6);

qdtraj  = zeros(size(qtraj));
qddtraj = zeros(size(qtraj));

for j = 1:6
    qdtraj(:,j)  = gradient(qtraj(:,j),  dt);
    qddtraj(:,j) = gradient(qdtraj(:,j), dt);
end

qddmax_traj = max(abs(qddtraj));

scale = max((qddmax_traj./qddmax)*fs);

for i=1:N

    J = R.jacob0(qtraj(i,:));

    for j=1:6

        Jv_row = J(1:3,j);
        Jw_row = J(4:6,j);

        nv = norm(Jv_row);
        nw = norm(Jw_row);

        scale_v_all(i,j) = (nv * qdmax(j)) / vmax;
        scale_w_all(i,j) = (nw * qdmax(j)) / wmax;
        scale_a_all(i,j)     = (nv * qddmax(j)) / amax;
        scale_alpha_all(i,j) = (nw * qddmax(j)) / alphamax;

    end
end

scale_v     = max(scale_v_all,     [], 'all');
scale_w     = max(scale_w_all,     [], 'all');
scale_a     = max(scale_a_all,     [], 'all');
scale_alpha = max(scale_alpha_all, [], 'all');

scale_2 = max([1, scale_v, scale_w, scale_a, scale_alpha]);

scale = max([scale,scale_2]);

end



