function [] = test_gen_traj(R)

    cfg = config();

    d = pi/180;
    fps = cfg.traj_fps;
    dt = cfg.traj_dt;

    cpoints_modes = [
        0.15 0.15 0.15 10*d 5*d 20*d 1
    ];

    q0 = [0, pi/3, pi/3, pi/3, pi/3, pi/3];

    qdmax = [180 180 180 360 360 360]*d;
    qddmax = [800 800 800 800 800 800]*d;

    q_traj = gen_traj(R, cpoints_modes, q0, qdmax, qddmax, 0, 1, 1, 2*pi, 2*pi);

    

    % R.plot(q_traj,'workspace',[-1,1,-1,1,-1,1],'trail',{'r','LineWidth',2}, 'scale',0.5);
    
    myAnimate(R,q_traj);

    qd_traj  = zeros(size(q_traj));
    qdd_traj = zeros(size(q_traj));
    for j = 1:size(q_traj,2)
        qd_traj(:,j)  = gradient(q_traj(:,j), dt);  
        qdd_traj(:,j) = gradient(qd_traj(:,j), dt);  
    end

    grafQ(q_traj,qd_traj,qdd_traj)

end