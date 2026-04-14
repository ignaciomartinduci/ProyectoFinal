function [] = tarea(R)

    cfg = config();
    dt = cfg.traj_dt;
    qdmax = cfg.qdmax;
    qddmax = cfg.qddmax;
    vmax = cfg.vmax;
    amax = cfg.amax;
    wmax = cfg.wmax;
    alphamax = cfg.alphamax;

    d = pi/180;
    e=0.01;

    caja_origen_superior =      [-0.22, 0.3,    0.2     0*d+e 180*d+e 0*d+e];
    caja_origen_inferior =      [-0.22,  0.3,    0.01    0*d+e 180*d+e 0*d+e];
    aux_1 =                     [-0.1,  0.3,    0.22    0*d+e 180*d+e 0*d+e];
    punto_medio =               [0,     0.3,   0.25    0*d+e 180*d+e 0*d+e];
    aux_2 =                     [0.1,  0.3,    0.22    0*d+e 180*d+e 0*d+e];
    caja_destino_superior =     [0.22,  0.3,    0.2     0*d+e 180*d+e 0*d+e];
    caja_destino_inferior =     [0.22,  0.3,    0.01    0*d+e 180*d+e 0*d+e];

    q_punto_medio = [1.0439    0.4714    1.0158    3.2389    1.5673   -0.5370];

    % Comienzo desde punto aleatorio
    q1 =  R.qlim(1,1) + (R.qlim(1,2)-R.qlim(1,1))*rand;
    q2 =  R.qlim(2,1) + (R.qlim(2,2)-R.qlim(2,1))*rand;
    q3 =  R.qlim(3,1) + (R.qlim(3,2)-R.qlim(3,1))*rand;
    q4 =  R.qlim(4,1) + (R.qlim(4,2)-R.qlim(4,1))*rand;
    q5 =  R.qlim(5,1) + (R.qlim(5,2)-R.qlim(5,1))*rand;
    q6 =  -2*pi + (2*pi-2*pi)*rand;
    
    q0 = q_punto_medio;
   % -1.0600    0.2519   -1.8082   -0.0185    1.5844   -2.6408
   % -1.0600   -1.4227    1.8082   -1.9602    1.5844   -2.6408
   % -1.0600   -0.5988   -0.8019    2.9676   -1.5844    0.5008
   % -1.0600   -1.3561    0.8019    2.1210   -1.5844    0.5008
   %  1.0439    1.3607   -0.8079   -2.1100    1.5673   -0.5370
   %  1.0439    0.5979    0.8079   -2.9628    1.5673   -0.5370
   %  1.0439    1.4190   -1.8038    1.9692   -1.5673    2.6046
   %  1.0439   -0.2518    1.8038    0.0325   -1.5673    2.6046
   % 
   % q0 = q_punto_medio;
    trayectoria = [
    punto_medio 0 %
    % aux_1 0 %
    caja_origen_superior 0 %
    caja_origen_inferior 1 
    caja_origen_superior 1 
    % % aux_1 0 %
    punto_medio 0 %
    % % aux_2 0 %
    caja_destino_superior 0 %
    caja_destino_inferior 1 
    caja_destino_superior 1 
    % % aux_2 0 %
    punto_medio 0
    ]; 

    q_traj = gen_traj(R, trayectoria, q0, qdmax, qddmax, 0, vmax, amax, wmax, alphamax);
    
    %% Visualización
    myAnimate(R,q_traj);

    % grafQaE(R, q_traj);
    % 
    % qd_traj  = zeros(size(q_traj));
    % qdd_traj = zeros(size(q_traj));
    % for j = 1:size(q_traj,2)
    %     qd_traj(:,j)  = gradient(q_traj(:,j), dt);  
    %     qdd_traj(:,j) = gradient(qd_traj(:,j), dt);  

    % grafQ(q_traj,qd_traj,qdd_traj);


end