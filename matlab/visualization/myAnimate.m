% myAnimate  Anima la trayectoria articular de un robot con visualización diagnóstica
%
% SINTAXIS
%   myAnimate(R, q_traj, cad_path)
%
% DESCRIPCIÓN
%   Reproduce la trayectoria articular q_traj sobre el modelo SerialLink R
%   mostrando simultáneamente tres paneles:
%
%     [ARTICULACIONES] | [ROBOT 3D] | [JACOBIANO]
%
%   Panel izquierdo  – Indica, para cada articulación, la posición normalizada
%                      en su rango de movimiento. La zona de color verde claro
%                      corresponde a la región central (bien alejada de límites)
%                      y se vuelve roja cuando el ángulo se acerca a un extremo.
%
%   Panel central    – Animación 3D del robot con modelos CAD, rastro de la
%                      trayectoria del efector final y cámara fija.
%
%   Panel derecho    – Número de condición κ(J) del Jacobiano geométrico en
%                      escala logarítmica. El color va de verde (bien condicionado)
%                      a rojo (cerca de singularidad). Si κ(J) > 100 aparece la
%                      advertencia "! SINGULAR".
%
% ENTRADAS
%   R        - Objeto SerialLink (Robotics Toolbox) que representa el robot.
%              Se usa R.jacob0, R.n, R.qlim, R.plot3d y R.animate.
%   q_traj   - Matriz (M × n_joints) de ángulos articulares en rad,
%              muestreada a cfg.traj_fps (por defecto 120 Hz).
%   cad_path - Cadena con la ruta al directorio que contiene los archivos
%              STL/CAD para plot3d.
%
% PARÁMETROS INTERNOS (desde config.m)
%   traj_fps  = 120 Hz  – Frecuencia a la que fue generada q_traj.
%   anim_fps  =  30 Hz  – Frecuencia de renderizado de la animación.
%   anim_dt   = 1/30 s  – Pausa entre cuadros (pause real-time).
%
% MATEMÁTICA RELEVANTE
%   Diezmado temporal
%     step = floor(traj_fps / anim_fps)
%     Se toma 1 de cada `step` filas de q_traj → sub-muestreo por factor 4.
%     Resultado: q_sub (N × n_joints), con N ≈ M/4 cuadros.
%
%   Jacobiano geométrico en la base (J ∈ ℝ^{6×n})
%     J = R.jacob0(q)  →  v_ee = J · q̇
%     Las 3 primeras filas mapean velocidades articulares a velocidad lineal
%     del efector; las 3 últimas a velocidad angular.
%     J depende de la configuración q (no-lineal en general).
%
%   Descomposición en Valores Singulares (SVD)
%     J = U · Σ · V^T,   Σ = diag(σ_1 ≥ σ_2 ≥ … ≥ σ_n ≥ 0)
%     Los valores singulares {σ_i} miden cuánto "amplifica" el Jacobiano
%     movimientos articulares en cada dirección del espacio de la tarea.
%     σ_min → 0 indica que existe una dirección del espacio de la tarea
%     que ninguna combinación de velocidades articulares puede alcanzar
%     de forma eficiente: el robot está en (o cerca de) una singularidad.
%
%   Número de condición
%     κ(J) = σ_max / σ_min
%     κ = 1  →  isótropo, el robot puede moverse igual de bien en todas
%               las direcciones → configuración óptima.
%     κ → ∞  →  singularidad cinemática; la inversa J^{-1} es numéricamente
%               inestable y los torques pueden dispararse.
%     Umbral de advertencia en este código: κ > 100.

function [] = myAnimate(R, q_traj, cad_path)

% =========================================================
% Parámetros de muestreo
% =========================================================
% La trayectoria fue generada a traj_fps Hz. La animación corre a anim_fps Hz.
% step = 4 (para 120 Hz / 30 Hz), es decir, se descarta 3 de cada 4 puntos.
% Esto reduce el costo de renderizado sin degradar la percepción visual
% dado que el ojo humano no distingue diferencias a más de ~24 fps.
cfg  = config();
step  = floor(cfg.traj_fps / cfg.anim_fps);
q_sub = q_traj(1:step:end, :);   % (N × n_joints) sub-muestreado
N     = size(q_sub, 1);          % cantidad de cuadros a renderizar

% Workspace (espacio de trabajo visible) en metros: [xmin xmax ymin ymax zmin zmax]
ws = [-0.6 0.6 -0.6 0.6 -0.1 0.8];

% =========================================================
% Pre-cómputo: Jacobiano y análisis de singularidades
% =========================================================
% Se calcula UNA SOLA VEZ antes de la animación para no bloquear el loop
% de rendering con álgebra lineal costosa frame a frame.
%
% Para cada configuración q_sub(i,:):
%   1. Se obtiene J ∈ ℝ^{6×6} (Jacobiano geométrico en la base, frame {0}).
%   2. Se factoriza  J = U·Σ·V^T  (SVD completa).
%   3. Se extraen  σ_max = sv(1)  y  σ_min = sv(end).
%   4. κ(J) = σ_max / σ_min  (número de condición en la norma 2).
%   5. σ_min se guarda por separado como indicador directo de singularidad.
cond_vals = zeros(N, 1);
smin_vals = zeros(N, 1);
for i = 1:N
    J             = R.jacob0(q_sub(i,:));  % J ∈ ℝ^{6×6}
    sv            = svd(J);                % sv ordenado descendente: σ_1 ≥ … ≥ σ_6
    cond_vals(i)  = max(sv) / min(sv);    % κ(J) = σ_max / σ_min
    smin_vals(i)  = min(sv);              % σ_min: proximidad a la singularidad
end

% Rango del eje logarítmico del panel derecho.
% cond_lo = 1 (mínimo teórico: matriz perfectamente isótropa).
% cond_hi se ajusta al peor caso observado en la trayectoria, con un mínimo
% de 1000 para que la escala sea legible incluso si el robot está bien condicionado.
cond_lo     = 1;
cond_hi     = max(max(cond_vals) * 1.5, 1000);

% Umbral a partir del cual se considera que el robot está en singularidad
% "práctica" (no necesariamente κ → ∞, pero numéricamente problemático).
sing_thresh = 100;

% Parámetros del panel de articulaciones
n_joints  = R.n;
qlim      = R.qlim;   % (n_joints × 2): [q_min, q_max] por fila, en rad
warn_frac = 0.10;     % El 10% más cercano a cada límite se colorea en rojo
bar_h     = 0.65;     % Altura visual de cada barra articular (en unidades del eje Y)

% =========================================================
% Figura con tres paneles
% =========================================================
% Las posiciones se dan como [left bottom width height] en fracción de la figura.
%   ax_joints: franja izquierda (13% del ancho)
%   ax_robot:  centro          (62% del ancho)
%   ax_bar:    franja derecha  (13% del ancho, centrada verticalmente)
figure;
set(gcf, 'Renderer', 'opengl');   % aceleración por hardware para las mallas STL
ax_joints = axes('Position', [0.01 0.06 0.13 0.86]);
ax_robot  = axes('Position', [0.16 0.06 0.62 0.86]);
ax_bar    = axes('Position', [0.83 0.22 0.13 0.62]);

% =========================================================
% Panel izquierdo: articulaciones
% =========================================================
% Cada articulación j (j = 1..n_joints) se representa como una barra horizontal.
% El eje X del panel va de 0 a 1: representa la posición normalizada
%   q_norm = (q - q_min) / (q_max - q_min)  ∈ [0, 1]
% El eje Y indexa las articulaciones (1 arriba, n_joints abajo, con YDir=reverse).
hold(ax_joints, 'on');
set(ax_joints, ...
    'YDir',       'reverse', ...         % q_1 en la parte superior
    'XLim',       [0 1], ...
    'YLim',       [0.5 n_joints + 0.5], ...
    'XTick',      [0 0.5 1], ...
    'XTickLabel', {'min','','max'}, ...   % 0=mínimo articular, 1=máximo articular
    'YTick',      1:n_joints, ...
    'YTickLabel', {'q_1','q_2','q_3','q_4','q_5','q_6'}, ...
    'Color',      [0.95 0.95 0.95], ...
    'Box',        'on', ...
    'FontSize',   8);
title(ax_joints, 'ARTICULACIONES', 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');

% hJMarkers(j): handle al marcador de posición de la articulación j.
% Se actualiza en cada frame del loop de animación.
hJMarkers = gobjects(n_joints, 1);

for j = 1:n_joints
    % Coordenadas verticales de la barra de la articulación j
    yc  = j;
    ylo = yc - bar_h/2;
    yhi = yc + bar_h/2;
    
    % Posición inicial normalizada de la articulación j
    q_norm = joint_norm(q_sub(1,j), qlim(j,:));
    
    % Zona roja en el extremo izquierdo: q_norm ∈ [0, warn_frac]
    % Indica que la articulación está cerca de su límite mínimo.
    patch(ax_joints, [0 warn_frac warn_frac 0], [ylo ylo yhi yhi], ...
        [1 0.80 0.80], 'EdgeColor', 'none');
    
    % Zona roja en el extremo derecho: q_norm ∈ [1-warn_frac, 1]
    % Indica que la articulación está cerca de su límite máximo.
    patch(ax_joints, [1-warn_frac 1 1 1-warn_frac], [ylo ylo yhi yhi], ...
        [1 0.80 0.80], 'EdgeColor', 'none');
    
    % Track de fondo: rectángulo gris que ocupa todo el rango [0,1]
    patch(ax_joints, [0 1 1 0], [ylo ylo yhi yhi], ...
        [0.82 0.82 0.82], 'EdgeColor', [0.55 0.55 0.55], 'LineWidth', 0.5);
    
    % Línea vertical en q_norm = 0.5: posición media del rango articular.
    % Referencia visual para saber si la articulación está centrada.
    plot(ax_joints, [0.5 0.5], [ylo yhi], '--', ...
        'Color', [0.65 0.65 0.65], 'LineWidth', 0.8);
    
    % Marcador dinámico: línea vertical en la posición actual q_norm.
    % Su color varía según limit_t (verde=centro, rojo=límite).
    t0 = limit_t(q_norm, warn_frac);
    hJMarkers(j) = plot(ax_joints, [q_norm q_norm], [ylo yhi], '-', ...
        'Color', bar_color(t0), 'LineWidth', 4);
end

% =========================================================
% Panel central: robot 3D
% =========================================================
% R.plot3d renderiza el robot con modelos CAD en la configuración q_sub(1,:).
%   'workspace'  – define los límites del volumen visible
%   'trail'      – activa el rastro del TCP (extremo operacional) en rojo
%   'path'       – directorio con archivos STL de los eslabones
%   'delay',0    – sin retardo interno (el timing lo controla pause() abajo)
axes(ax_robot);
R.plot3d(q_sub(1,:), 'workspace', ws, 'trail', {'r','LineWidth',2}, 'path', cad_path, 'delay', 0);

% Se restaura ax_robot como eje activo porque plot3d puede crear ejes internos.
axes(ax_robot);
axis(ax_robot, 'equal');
xlim(ax_robot, ws(1:2));
ylim(ax_robot, ws(3:4));
zlim(ax_robot, ws(5:6));

% 'vis3d' congela la relación de aspecto durante las rotaciones interactivas,
% evitando que el robot "salte" visualmente al girar la cámara.
axis(ax_robot, 'vis3d');

% 'manual' impide que MATLAB reajuste automáticamente el ángulo de cámara
% entre frames, lo que causaría un efecto de "zoom pulsante".
set(ax_robot, 'CameraViewAngleMode', 'manual');

% Iluminación: 'headlight' sigue a la cámara; 'gouraud' interpola normales
% por vértice dando una superficie más suave que 'flat' shading.
camlight headlight;
lighting gouraud;

% =========================================================
% Panel derecho: Jacobiano (número de condición)
% =========================================================
% El panel muestra κ(J) como una barra que crece hacia arriba en escala
% logarítmica. El color va de verde (κ ≈ 1) a rojo (κ → ∞).
%
% Escala logarítmica: permite visualizar en el mismo eje rangos muy dispares.
% Por ejemplo, κ puede valer 2 (excelente) o 5000 (singularidad práctica).
% En escala lineal, el valor 2 sería invisible junto al 5000.
cval0 = cond_vals(1);
smin0 = smin_vals(1);
t0    = bar_t(cval0, cond_lo, cond_hi);

hold(ax_bar, 'on');

% Rectángulo de fondo gris que ocupa todo el rango [cond_lo, cond_hi]
patch(ax_bar, [0 1 1 0], [cond_lo cond_lo cond_hi cond_hi], ...
    [0.85 0.85 0.85], 'EdgeColor', 'none');

% Barra de color dinámica: su altura Y superior es cval (en escala log)
hBar  = patch(ax_bar, [0 1 1 0], [cond_lo cond_lo cval0 cval0], ...
    bar_color(t0), 'EdgeColor', 'none');

% Línea horizontal que marca el nivel exacto de cval
hLine = plot(ax_bar, [0 1], [cval0 cval0], 'k-', 'LineWidth', 1.5);

set(ax_bar, ...
    'YScale', 'log', ...     % eje Y logarítmico: cada década ocupa el mismo espacio
    'XTick',  [], ...        % sin marcas en X (el eje X no tiene significado aquí)
    'YColor', 'k', ...
    'XColor', 'k', ...
    'Color',  [0.95 0.95 0.95], ...
    'Box',    'on', ...
    'LineWidth', 1);
ylim(ax_bar, [cond_lo cond_hi]);
xlim(ax_bar, [0 1]);

title(ax_bar, 'JACOBIANO',  'Color', 'k', 'FontSize', 9, 'FontWeight', 'bold');
ylabel(ax_bar, 'cond(J)',   'Color', 'k', 'FontSize', 8);

% xlabel se reutiliza como cuadro de texto numérico (truco de layout):
% muestra κ(J) redondeado y σ_min con 4 decimales.
hInfo = xlabel(ax_bar, sprintf('cond: %.0f\nsmin: %.4f', cval0, smin0), ...
    'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold');

% Texto de advertencia de singularidad (visible sólo cuando κ > sing_thresh).
% 'Units','normalized' → posición relativa al panel, independiente de la escala Y.
hWarn = text(ax_bar, 0.5, 0.65, '! SINGULAR', ...
    'Units',              'normalized', ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment',  'middle', ...
    'Color',              [0.85 0 0], ...
    'FontSize',           8, ...
    'FontWeight',         'bold', ...
    'Rotation',           90, ...
    'Clipping',           'off', ...
    'Visible',            'off');

% =========================================================
% Loop de animación
% =========================================================
% En cada iteración i:
%   1. R.animate mueve el modelo 3D a la configuración q_sub(i,:).
%   2. Se actualizan los marcadores articulares (panel izquierdo).
%   3. Se actualiza la barra de condición del Jacobiano (panel derecho).
%   4. drawnow fuerza el redibujado inmediato del buffer gráfico.
%   5. pause(anim_dt) sincroniza la velocidad de reproducción con anim_fps.
%
% Nota: se usa set() para modificar propiedades de handles existentes en lugar
% de borrar y recrear objetos gráficos, lo que es mucho más rápido (~10×).
for i = 1:N

    t_frame = tic;   % marca inicio del frame para controlar el timing real

    % Mueve el robot al frame i en el panel central
    R.animate(q_sub(i,:));
    
    % -- Panel izquierdo: actualizar cada marcador articular --
    for j = 1:n_joints
        % Normalización:  q_norm = (q - q_min) / (q_max - q_min)  ∈ [0,1]
        q_norm = joint_norm(q_sub(i,j), qlim(j,:));
        
        % Distancia al límite más cercano, mapeada a [0,1]:
        % t = 0 → articulación en el centro del rango  → color verde
        % t = 1 → articulación tocando un límite        → color rojo
        t = limit_t(q_norm, warn_frac);
        
        % XData fija la posición horizontal del marcador; Color cambia según t
        set(hJMarkers(j), 'XData', [q_norm q_norm], 'Color', bar_color(t));
    end
    
    % -- Panel derecho: actualizar barra de condición del Jacobiano --
    cval = cond_vals(i);
    smin = smin_vals(i);
    
    % Mapeo logarítmico de cval al intervalo [0,1]:
    % t = (log10(κ) - log10(κ_min)) / (log10(κ_max) - log10(κ_min))
    % t = 0 → κ = 1 (isótropo) → verde
    % t = 1 → κ = cond_hi       → rojo
    t = bar_t(cval, cond_lo, cond_hi);
    
    % YData: borde inferior fijo en cond_lo, borde superior en cval (log scale)
    set(hBar,  'YData', [cond_lo cond_lo cval cval], 'FaceColor', bar_color(t));
    set(hLine, 'YData', [cval cval]);
    set(hInfo, 'String', sprintf('cond: %.0f\nsmin: %.4f', cval, smin));
    
    % Advertencia de singularidad práctica
    if cval > sing_thresh
        set(hWarn, 'Visible', 'on');
    else
        set(hWarn, 'Visible', 'off');
    end
    
    drawnow;

    % Pausa solo el tiempo restante del presupuesto del frame.
    % Si el render tardó más que anim_dt, no pausa y el loop sigue de inmediato.
    remaining = cfg.anim_dt - toc(t_frame);
    if remaining > 0
        pause(remaining);
    end
end

end % myAnimate

% =========================================================
% Funciones auxiliares (locales al archivo)
% =========================================================

% joint_norm  Normaliza un ángulo articular al intervalo [0, 1]
%
%   q_norm = (q - q_min) / (q_max - q_min)
%
%   El resultado se satura a [0,1] con max/min para evitar desbordamientos
%   si q excede los límites nominales (posible en la primera/última muestra
%   de la trayectoria por efectos numéricos de la interpolación).
function n = joint_norm(q, lim)
n = (q - lim(1)) / (lim(2) - lim(1));
n = max(0, min(1, n));
end

% limit_t  Mapea la posición normalizada a un "nivel de alarma" t ∈ [0, 1]
%
%   margin = distancia normalizada al límite más cercano:
%     margin = min(q_norm, 1 - q_norm)  ∈ [0, 0.5]
%     → margin = 0    si q_norm ∈ {0, 1}  (en el límite)
%     → margin = 0.5  si q_norm = 0.5     (en el centro)
%
%   t = 1 - margin / warn_frac
%     → t = 1  si margin = 0            (en el límite)        → rojo
%     → t = 0  si margin ≥ warn_frac    (zona segura)         → verde
%     → t ∈ (0,1) dentro de la zona de advertencia [0, warn_frac]
%
%   La saturación con max(0,...) garantiza t ≥ 0 en la zona central.
function t = limit_t(q_norm, warn_frac)
margin = min(q_norm, 1 - q_norm);
t = max(0, min(1, 1 - margin / warn_frac));
end

% bar_t  Mapea el número de condición κ a t ∈ [0, 1] en escala logarítmica
%
%   Se trabaja en escala log porque κ puede variar en varios órdenes de magnitud.
%   La transformación es:
%     t = (log10(κ) - log10(κ_min)) / (log10(κ_max) - log10(κ_min))
%
%   Propiedades:
%     t = 0  si κ = κ_min = 1  (robot isótropo, condición perfecta) → verde
%     t = 1  si κ ≥ κ_max      (robot en singularidad práctica)      → rojo
%
%   El  max(cval, cond_lo)  evita log10(0) para κ < 1 (no ocurre en la práctica
%   ya que κ ≥ 1 siempre, pero protege ante errores numéricos de SVD).
%   El  min(..., 1)  satura t en 1 si κ supera cond_hi.
function t = bar_t(cval, cond_lo, cond_hi)
t = min( ...
    (log10(max(cval, cond_lo)) - log10(cond_lo)) / ...
    (log10(cond_hi)            - log10(cond_lo)), ...
    1);
end

% bar_color  Interpola linealmente entre verde puro y rojo puro
%
%   Entrada:  t ∈ [0, 1]
%   Salida:   c = [R, G, B] en el espacio de color RGB normalizado [0,1]^3
%
%   Mapeo:
%     t = 0  →  c = [0, 1, 0]  (verde)
%     t = 1  →  c = [1, 0, 0]  (rojo)
%     t = 0.5 → c = [0.5, 0.5, 0]  (amarillo-oliva)
%
%   Nota: el canal azul B = 0 siempre. La interpolación es lineal en RGB,
%   no en espacio perceptual (HSV/LAB), por lo que el punto medio visual
%   no es exactamente amarillo puro, sino un amarillo-verdoso. Para
%   aplicaciones donde la percepción exacta sea crítica, considerar HSV.
function c = bar_color(t)
c = [t, 1-t, 0];
end
