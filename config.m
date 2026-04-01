function cfg = config()

    % Cálculo de trayectorias

        cfg.traj_fps=30;
        cfg.traj_dt=1/cfg.traj_fps;
        cfg.s_dot_max = 1.875; % Propiedad de perfil quintic
        cfg.s_ddot_max = 7.5; % Propiedad de perfil quintic

    % Visualización y animaciones

        cfg.anim_fps = 30;
        cfg.anim_dt = 1/cfg.anim_fps;

end