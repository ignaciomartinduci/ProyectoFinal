function [] = myAnimate(R, q_traj, cad_path)

cfg  = config();
step = floor(cfg.traj_fps / cfg.anim_fps);
q_sub = q_traj(1:step:end, :);
N     = size(q_sub, 1);

ws = [-0.6 0.6 -0.6 0.6 -0.1 0.8];

% --- Precómputo: Jacobiano y valores singulares ---
cond_vals = zeros(N, 1);
smin_vals = zeros(N, 1);
for i = 1:N
    J   = R.jacob0(q_sub(i,:));
    sv  = svd(J);
    cond_vals(i) = max(sv) / min(sv);
    smin_vals(i) = min(sv);
end

cond_lo     = max(1, min(cond_vals));
cond_hi     = max(cond_vals) * 1.5;
sing_thresh = cond_hi / 3;

n_joints  = R.n;
qlim      = R.qlim;           % n×2: [q_min, q_max]
warn_frac = 0.10;             % zona de advertencia: 10% de cada extremo
bar_h     = 0.65;             % altura de cada barra articular

% --- Figura con tres paneles ---
figure;
ax_joints = axes('Position', [0.01 0.06 0.13 0.86]);
ax_robot  = axes('Position', [0.16 0.06 0.62 0.86]);
ax_bar    = axes('Position', [0.83 0.22 0.13 0.62]);

% =========================================================
% Panel izquierdo: articulaciones
% =========================================================
hold(ax_joints, 'on');
set(ax_joints, ...
    'YDir',   'reverse', ...
    'XLim',   [0 1], ...
    'YLim',   [0.5 n_joints + 0.5], ...
    'XTick',  [0 0.5 1], ...
    'XTickLabel', {'min','','max'}, ...
    'YTick',  1:n_joints, ...
    'YTickLabel', {'q_1','q_2','q_3','q_4','q_5','q_6'}, ...
    'Color',  [0.95 0.95 0.95], ...
    'Box',    'on', ...
    'FontSize', 8);
title(ax_joints, 'ARTICULACIONES', 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');

hJMarkers = gobjects(n_joints, 1);

for j = 1:n_joints
    yc     = j;
    ylo    = yc - bar_h/2;
    yhi    = yc + bar_h/2;
    q_norm = joint_norm(q_sub(1,j), qlim(j,:));

    % Zona de advertencia: extremo izquierdo (cerca del mínimo)
    patch(ax_joints, [0 warn_frac warn_frac 0], [ylo ylo yhi yhi], ...
        [1 0.80 0.80], 'EdgeColor', 'none');

    % Zona de advertencia: extremo derecho (cerca del máximo)
    patch(ax_joints, [1-warn_frac 1 1 1-warn_frac], [ylo ylo yhi yhi], ...
        [1 0.80 0.80], 'EdgeColor', 'none');

    % Track de fondo
    patch(ax_joints, [0 1 1 0], [ylo ylo yhi yhi], ...
        [0.82 0.82 0.82], 'EdgeColor', [0.55 0.55 0.55], 'LineWidth', 0.5);

    % Línea central de referencia (posición media del rango)
    plot(ax_joints, [0.5 0.5], [ylo yhi], '--', ...
        'Color', [0.65 0.65 0.65], 'LineWidth', 0.8);

    % Marcador de posición actual
    t0 = limit_t(q_norm, warn_frac);
    hJMarkers(j) = plot(ax_joints, [q_norm q_norm], [ylo yhi], '-', ...
        'Color', bar_color(t0), 'LineWidth', 4);
end

% =========================================================
% Panel central: robot
% =========================================================
axes(ax_robot);
R.plot3d(q_sub(1,:), 'workspace', ws, 'trail', {'r','LineWidth',2}, 'path', cad_path, 'delay', 0);
axes(ax_robot);
axis(ax_robot, 'equal');
xlim(ax_robot, ws(1:2));
ylim(ax_robot, ws(3:4));
zlim(ax_robot, ws(5:6));
axis(ax_robot, 'vis3d');
set(ax_robot, 'CameraViewAngleMode', 'manual');
camlight headlight;
lighting gouraud;

% =========================================================
% Panel derecho: Jacobiano
% =========================================================
cval0 = cond_vals(1);
smin0 = smin_vals(1);
t0    = bar_t(cval0, cond_lo, cond_hi);

hold(ax_bar, 'on');
patch(ax_bar, [0 1 1 0], [cond_lo cond_lo cond_hi cond_hi], [0.85 0.85 0.85], 'EdgeColor', 'none');
hBar  = patch(ax_bar, [0 1 1 0], [cond_lo cond_lo cval0 cval0], bar_color(t0), 'EdgeColor', 'none');
hLine = plot(ax_bar, [0 1], [cval0 cval0], 'k-', 'LineWidth', 1.5);

set(ax_bar, 'YScale', 'log', 'XTick', [], 'YColor', 'k', 'XColor', 'k', ...
    'Color', [0.95 0.95 0.95], 'Box', 'on', 'LineWidth', 1);
ylim(ax_bar, [cond_lo cond_hi]);
xlim(ax_bar, [0 1]);

title(ax_bar, 'JACOBIANO', 'Color', 'k', 'FontSize', 9, 'FontWeight', 'bold');
ylabel(ax_bar, 'cond(J)', 'Color', 'k', 'FontSize', 8);
hInfo = xlabel(ax_bar, sprintf('cond: %.0f\nsmin: %.4f', cval0, smin0), ...
    'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');

hWarn = text(ax_bar, 0.5, 0.65, '! SINGULAR', 'Units', 'normalized', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'Color', [0.85 0 0], 'FontSize', 8, 'FontWeight', 'bold', ...
    'Rotation', 90, 'Clipping', 'off', 'Visible', 'off');

% =========================================================
% Loop de animación
% =========================================================
for i = 1:N
    R.animate(q_sub(i,:));

    % -- Articulaciones --
    for j = 1:n_joints
        q_norm = joint_norm(q_sub(i,j), qlim(j,:));
        t      = limit_t(q_norm, warn_frac);
        set(hJMarkers(j), 'XData', [q_norm q_norm], 'Color', bar_color(t));
    end

    % -- Jacobiano --
    cval = cond_vals(i);
    smin = smin_vals(i);
    t    = bar_t(cval, cond_lo, cond_hi);

    set(hBar,  'YData', [cond_lo cond_lo cval cval], 'FaceColor', bar_color(t));
    set(hLine, 'YData', [cval cval]);
    set(hInfo, 'String', sprintf('cond: %.0f\nsmin: %.4f', cval, smin));

    if cval > sing_thresh
        set(hWarn, 'Visible', 'on');
    else
        set(hWarn, 'Visible', 'off');
    end

    drawnow;
    pause(cfg.anim_dt);
end

end

% Normaliza q al rango [0,1] según los límites articulares
function n = joint_norm(q, lim)
    n = (q - lim(1)) / (lim(2) - lim(1));
    n = max(0, min(1, n));
end

% t en [0,1]: 0 = lejos del límite (verde), 1 = en el límite (rojo)
function t = limit_t(q_norm, warn_frac)
    margin = min(q_norm, 1 - q_norm);
    t = max(0, min(1, 1 - margin / warn_frac));
end

% t en [0,1]: 0 = bien condicionado (verde), 1 = singular (rojo)
function t = bar_t(cval, cond_lo, cond_hi)
    t = min((log10(max(cval, cond_lo)) - log10(cond_lo)) / (log10(cond_hi) - log10(cond_lo)), 1);
end

function c = bar_color(t)
    c = [t, 1-t, 0];
end
