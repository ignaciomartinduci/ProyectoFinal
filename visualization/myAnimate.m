function [] = myAnimate(R, q_traj)

cfg = config();
step = (cfg.traj_fps/cfg.anim_fps);

path = fullfile(pwd, 'models', 'CAD');

ws = [-1 1 -1 1 -1 1];
figure;
R.plot3d(q_traj(1,:), 'workspace', ws,'trail',{'r','LineWidth',2}, 'path', path);
camva(20);
R.plot3d(q_traj(1:step:end,:), 'workspace', ws, 'trail',{'r','LineWidth',2}, 'path',path, 'delay', cfg.anim_dt);


end