function [] = test_gen_traj(R, cad_path)

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

[q_traj, dbg] = gen_traj(R, cpoints_modes, q0, qdmax, qddmax, 0, vmax, amax, wmax, alphamax);

myAnimate(R, q_traj, cad_path);

grafQaE(R, q_traj);

qd_traj  = zeros(size(q_traj));
qdd_traj = zeros(size(q_traj));
for j = 1:size(q_traj,2)
    qd_traj(:,j)  = gradient(q_traj(:,j), dt);
    qdd_traj(:,j) = gradient(qd_traj(:,j), dt);
end

grafQ(q_traj,qd_traj,qdd_traj)

grafScaling(R, dbg, qdmax, vmax, amax, wmax, alphamax)

%% Verificación de paso por puntos de control

N = size(q_traj,1);
N_cpoints = size(cpoints_modes,1);
N_alcanzados = 0;

for i = 1:N
    T_i = R.fkine(q_traj(i,:));
    
    for j = 1:N_cpoints
        T_cpoint = transl(cpoints_modes(j,1), cpoints_modes(j,2), cpoints_modes(j,3)) * ...
            rpy2tr(cpoints_modes(j,4), cpoints_modes(j,5), cpoints_modes(j,6));
        R_i    = [T_i.n, T_i.o, T_i.a];
        pos_ok = norm(T_i.t - T_cpoint(1:3,4))        < e;
        rot_ok = norm(R_i   - T_cpoint(1:3,1:3), 'fro') < e; % Norma frobenius, similar a euclidiana pero tiene pasos previos para llevar la matriz a un vector.
        if pos_ok && rot_ok
            N_alcanzados = N_alcanzados + 1;
            break
        end
    end
    
end
N_alcanzados
if N_alcanzados >= N_cpoints
    disp("Todos los puntos de control fueron alcanzados.");
else
    disp("No se alcanzaron todos los puntos de control.");
end



end