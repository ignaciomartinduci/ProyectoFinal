function [] = myAnimate(R, q_traj, delay)
    
    ws = [-1 1 -1 1 -1 1];
    figure;
    R.plot(q_traj(1,:), 'workspace', ws,'trail',{'r','LineWidth',2}, 'scale',0.5);
    R.plot(q_traj, 'workspace', ws, 'trail',{'r','LineWidth',2}, 'delay', delay);

end