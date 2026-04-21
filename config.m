function cfg = config()

% Cálculo de trayectorias

cfg.traj_fps=120;
cfg.traj_dt=1/cfg.traj_fps;
cfg.s_dot_max = 1.875; % Propiedad de perfil quintic
cfg.s_ddot_max = 7.5; % Propiedad de perfil quintic

% Visualización y animaciones

cfg.anim_fps = 30;
cfg.anim_dt = 1/cfg.anim_fps;

% Otros parámetros del robot

cfg.qdmax = ones(1,6)*pi;
cfg.qddmax = ones(1,6)*13.9626;
cfg.vmax = 1;
cfg.amax = 1.5;
cfg.wmax = 3;
cfg.alphamax = 3;


end