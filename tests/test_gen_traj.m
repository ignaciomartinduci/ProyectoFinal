function [] = test_gen_traj(R)

cfg = config();

d = pi/180;
fps = cfg.traj_fps;
dt = cfg.traj_dt;
qdmax = cfg.qdmax;
qddmax = cfg.qddmax;
vmax = cfg.vmax;
amax = cfg.amax;
wmax = cfg.wmax;
alphamax = cfg.alphamax;

e=0.01;

cpoints_modes = [
    0.1,  0.3,    0.22    0*d+e 180*d+e 0*d+e 0
    0.22,  0.3,    0.2     0*d+e 180*d+e 0*d+e 0
    ];

q0 = [1.3913    0.5478    1.0215    3.1548    1.5629   -0.1896];

qdmax = [180 180 180 360 360 360]*d;
qddmax = [800 800 800 800 800 800]*d;

q_traj = gen_traj(R, cpoints_modes, q0, qdmax, qddmax, 0, vmax, amax, wmax, alphamax);



% R.plot(q_traj,'workspace',[-1,1,-1,1,-1,1],'trail',{'r','LineWidth',2}, 'scale',0.5);

myAnimate(R,q_traj);

grafQaE(R, q_traj);

qd_traj  = zeros(size(q_traj));
qdd_traj = zeros(size(q_traj));
for j = 1:size(q_traj,2)
    qd_traj(:,j)  = gradient(q_traj(:,j), dt);
    qdd_traj(:,j) = gradient(qd_traj(:,j), dt);
end

grafQ(q_traj,qd_traj,qdd_traj)

end