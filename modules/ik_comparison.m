function [] = ik_comparison(R, cad_path)
%
% Visualiza las variantes periódicas (±2π en q1, q2, q3) de una solución
% geométrica de la cinemática inversa para la pose de referencia:
%
%   q_ref = [0°, 45°, 45°, 45°, 45°, 45°]
%
% Genera 2^3 = 8 combinaciones (válidas dentro de qlim) y anima el robot
% moviéndose entre ellas en espacio articular.  Al llegar a cada posición
% se habilita el botón CONTINUAR en la figura.

cfg    = config();
dt     = cfg.traj_dt;
step   = floor(cfg.traj_fps / cfg.anim_fps);
qdmax  = cfg.qdmax;
t_acc  = max(qdmax ./ cfg.qddmax);

d  = pi / 180;
ws = [-0.6 0.6 -0.6 0.6 -0.1 0.8];

%% 1. FK de la pose de referencia
q_ref = [0, 45, 45, 45, 45, 45] * d;
T     = R.fkine(q_ref);
p     = transl(T);
rpy   = tr2rpy(T, 'zyx');
x = p(1);  y = p(2);  z = p(3);
alpha = rpy(1);  beta = rpy(2);  gamma = rpy(3);

fprintf('[IK_COMPARISON] Pose cartesiana: x=%.3f  y=%.3f  z=%.3f\n', x, y, z);

%% 2. IK y extracción de solución geométrica base
[ampl_all_sol, ~] = inv_kinematics(x, y, z, alpha, beta, gamma, q_ref, false, R, 'FORCE');

if isempty(ampl_all_sol)
    error('[IK_COMPARISON] La IK no encontró soluciones para la pose de referencia.');
end

% ampl_all_sol: 64 filas por solución geométrica.
% La fila 64*(i-1)+1 corresponde a la solución geométrica i sin shifts.
geo_sols  = ampl_all_sol(1:64:end, :);
costs     = sum(abs(wrapToPi(geo_sols - q_ref)), 2);
[~, best] = min(costs);
q_geo     = geo_sols(best, :);

fprintf('[IK_COMPARISON] Solución geométrica base (la más cercana a q_ref):\n');
fprintf('  [ ');  fprintf('%.1f°  ', q_geo * 180/pi);  fprintf(']\n');

%% 3. Variantes periódicas en q1, q2, q3 (mismo criterio que inv_kinematics)
%    Si q_geo(j) >= 0: alternativa = q_geo(j) - 2π
%    Si q_geo(j) <  0: alternativa = q_geo(j) + 2π
q_alt = q_geo;
for j = 1:3
    if q_geo(j) >= 0
        q_alt(j) = q_geo(j) - 2*pi;
    else
        q_alt(j) = q_geo(j) + 2*pi;
    end
end

solutions  = [];
sol_labels = {};

for combo = 0:7
    bits   = dec2bin(combo, 3) - '0';   % [b1 b2 b3], MSB = q1
    q_cand = q_geo;
    for j = 1:3
        if bits(j)
            q_cand(j) = q_alt(j);
        end
    end

    if all(q_cand >= R.qlim(:,1)' & q_cand <= R.qlim(:,2)')
        solutions(end+1, :) = q_cand;           %#ok<AGROW>
        sol_labels{end+1}   = make_label(q_cand, q_geo, bits);  %#ok<AGROW>
    end
end

n_sol = size(solutions, 1);
fprintf('[IK_COMPARISON] Variantes válidas: %d / 8\n', n_sol);

if n_sol == 0
    error('[IK_COMPARISON] Ninguna variante periódica cae dentro de los límites articulares.');
end

%% 4. Figura
n_joints  = R.n;
qlim      = R.qlim;
warn_frac = 0.10;
bar_h     = 0.65;

fig = figure('Name', 'IK Comparison — Periodicidad Articular', ...
             'Color', [0.96 0.96 0.96], 'NumberTitle', 'off');

% Eje invisible para el título dinámico (arriba del robot)
ax_title  = axes(fig, 'Position', [0.16 0.93 0.82 0.06], 'Visible', 'off');
ax_joints = axes(fig, 'Position', [0.01 0.12 0.13 0.80]);
ax_robot  = axes(fig, 'Position', [0.16 0.12 0.82 0.80]);

h_title = text(ax_title, 0.5, 0.5, 'Iniciando...', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k', 'Units', 'normalized');

btn = uicontrol(fig, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.35 0.02 0.30 0.08], ...
    'String', 'CONTINUAR  →', 'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.20 0.72 0.20], 'ForegroundColor', 'w', ...
    'Enable', 'off', ...
    'Callback', @(~,~) set(fig, 'UserData', 'go'));

%% 5. Panel de articulaciones (izquierda)
hold(ax_joints, 'on');
set(ax_joints, ...
    'YDir',   'reverse', ...
    'XLim',   [0 1], 'YLim', [0.5 n_joints + 0.5], ...
    'XTick',  [0 0.5 1], 'XTickLabel', {'min','','max'}, ...
    'YTick',  1:n_joints, 'YTickLabel', {'q_1','q_2','q_3','q_4','q_5','q_6'}, ...
    'Color',  [0.95 0.95 0.95], 'Box', 'on', 'FontSize', 8);
title(ax_joints, {'ARTICULACIONES', '{\color[rgb]{0.2,0.45,0.85}■} con variantes periódicas'}, ...
    'FontSize', 7.5, 'FontWeight', 'bold', 'Color', 'k');

hJMarkers = gobjects(n_joints, 1);
for j = 1:n_joints
    ylo = j - bar_h/2;
    yhi = j + bar_h/2;
    q_n = joint_norm(solutions(1,j), qlim(j,:));

    % Zonas de advertencia en los extremos del rango
    patch(ax_joints, [0 warn_frac warn_frac 0],             [ylo ylo yhi yhi], [1.0 0.80 0.80], 'EdgeColor','none');
    patch(ax_joints, [1-warn_frac 1 1 1-warn_frac],         [ylo ylo yhi yhi], [1.0 0.80 0.80], 'EdgeColor','none');
    % Track de fondo
    patch(ax_joints, [0 1 1 0],                             [ylo ylo yhi yhi], [0.82 0.82 0.82], 'EdgeColor',[0.55 0.55 0.55], 'LineWidth',0.5);
    % Línea central de referencia (posición media del rango)
    plot(ax_joints, [0.5 0.5], [ylo yhi], '--', 'Color',[0.65 0.65 0.65], 'LineWidth',0.8);
    % Borde azul para q1, q2, q3: indica que tienen variante periódica
    if j <= 3
        plot(ax_joints, [0 1 1 0 0], [ylo ylo yhi yhi ylo], '-', ...
            'Color',[0.20 0.45 0.85], 'LineWidth', 2.0);
    end
    % Marcador de posición actual (verde → rojo según proximidad al límite)
    t0 = limit_t(q_n, warn_frac);
    hJMarkers(j) = plot(ax_joints, [q_n q_n], [ylo yhi], '-', ...
        'Color', bar_color(t0), 'LineWidth', 4);
end

%% 6. Setup del robot en la primera posición
axes(ax_robot);
R.plot3d(solutions(1,:), 'workspace', ws, 'path', cad_path, 'delay', 0);
axes(ax_robot);
axis(ax_robot, 'equal');
xlim(ax_robot, ws(1:2));  ylim(ax_robot, ws(3:4));  zlim(ax_robot, ws(5:6));
axis(ax_robot, 'vis3d');
set(ax_robot, 'CameraViewAngleMode', 'manual');
camlight headlight;
lighting gouraud;

%% 7. Loop principal: animar y esperar al usuario en cada solución
for k = 1:n_sol

    btn_travel(btn);
    set(h_title, 'String', sprintf('Viajando a solución %d / %d...', k, n_sol));
    drawnow;

    if k == 1
        % Primera posición: robot ya está ahí desde el setup
        R.animate(solutions(1,:));
        update_joints(hJMarkers, solutions(1,:), qlim, warn_frac);
    else
        % Trayectoria en espacio articular con mstraj (no pasa por IK)
        q_seg = mstraj(solutions(k,:), qdmax, [], solutions(k-1,:), dt, t_acc');
        for i = 1:step:size(q_seg, 1)
            if ~isvalid(fig); return; end
            R.animate(q_seg(i,:));
            update_joints(hJMarkers, q_seg(i,:), qlim, warn_frac);
            drawnow;
            pause(cfg.anim_dt);
        end
    end

    % Asegurar posición exacta final y actualizar título
    R.animate(solutions(k,:));
    update_joints(hJMarkers, solutions(k,:), qlim, warn_frac);
    set(h_title, 'String', sprintf('Solución %d / %d  —  %s', k, n_sol, sol_labels{k}));
    drawnow;

    % Habilitar botón o finalizar
    if k < n_sol
        btn_ready(btn);
        set(fig, 'UserData', '');        % resetear antes de esperar
        waitfor(fig, 'UserData', 'go');
        if ~isvalid(fig); return; end
    else
        btn_end(btn);
    end

end

end

%% =====================================================================
%  Funciones locales
%% =====================================================================

function btn_ready(btn)
    set(btn, 'Enable','on',  'String','CONTINUAR  →', ...
        'BackgroundColor',[0.20 0.72 0.20], 'ForegroundColor','w');
end

function btn_travel(btn)
    set(btn, 'Enable','off', 'String','Viajando...', ...
        'BackgroundColor',[0.74 0.74 0.74], 'ForegroundColor',[0.40 0.40 0.40]);
end

function btn_end(btn)
    set(btn, 'Enable','off', 'String','FIN', ...
        'BackgroundColor',[0.30 0.45 0.85], 'ForegroundColor','w');
end

function update_joints(hMarkers, q, qlim, warn_frac)
    for j = 1:length(q)
        q_n = joint_norm(q(j), qlim(j,:));
        set(hMarkers(j), 'XData', [q_n q_n], 'Color', bar_color(limit_t(q_n, warn_frac)));
    end
end

% Label para el título: indica si cada joint usa el valor base o la variante ±360°
function label = make_label(q_cand, q_geo, bits)
    names = {'q1', 'q2', 'q3'};
    parts = cell(1, 3);
    for j = 1:3
        val_deg = round(q_cand(j) * 180/pi);
        if bits(j) == 0
            parts{j} = sprintf('%s: %d° (base)', names{j}, val_deg);
        elseif q_cand(j) > q_geo(j)
            parts{j} = sprintf('%s: %d° (+360°)', names{j}, val_deg);
        else
            parts{j} = sprintf('%s: %d° (-360°)', names{j}, val_deg);
        end
    end
    label = strjoin(parts, '    ');
end

function n = joint_norm(q, lim)
    n = max(0, min(1, (q - lim(1)) / (lim(2) - lim(1))));
end

function t = limit_t(q_norm, warn_frac)
    t = max(0, min(1, 1 - min(q_norm, 1 - q_norm) / warn_frac));
end

function c = bar_color(t)
    c = [t, 1-t, 0];
end
