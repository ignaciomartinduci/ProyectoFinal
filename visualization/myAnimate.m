function [] = myAnimate(R, q_traj)
    
    cfg = config();
    step = (cfg.traj_fps/cfg.anim_fps);
    
    ws = [-1 1 -1 1 -1 1];
    figure;
    R.plot(q_traj(1,:), 'workspace', ws,'trail',{'r','LineWidth',2}, 'scale',0.5)
    R.plot(q_traj(1:step:end,:), 'workspace', ws, 'trail',{'r','LineWidth',2}, 'scale',0.5, 'delay', cfg.anim_dt);

end