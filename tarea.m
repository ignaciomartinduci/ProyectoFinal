function [] = tarea(R)

    d = pi/180;
    e=0.01;
    dt = 0.05;

    caja_origen_superior =      [-0.22, 0.3,    0.2     0*d+e 180*d+e 0*d+e];
    caja_origen_inferior =      [-0.22,  0.3,    0.01    0*d+e 180*d+e 0*d+e];
    aux_1 =                     [-0.1,  0.3,    0.22    0*d+e 180*d+e 0*d+e];
    punto_medio =               [0,     0.3,   0.25    0*d+e 180*d+e 0*d+e];
    aux_2 =                     [0.1,  0.3,    0.22    0*d+e 180*d+e 0*d+e];
    caja_destino_superior =     [0.22,  0.3,    0.2     0*d+e 180*d+e 0*d+e];
    caja_destino_inferior =     [0.22,  0.3,    0.01    0*d+e 180*d+e 0*d+e];

    q0 = [-1.0600    0.2519   -1.8082   -0.0185    1.5844   -2.6408]; 
   %     -1.0600    0.2519   -1.8082   -0.0185    1.5844   -2.6408
   % -1.0600   -1.4227    1.8082   -1.9602    1.5844   -2.6408
   % -1.0600   -0.5988   -0.8019    2.9676   -1.5844    0.5008
   % -1.0600   -1.3561    0.8019    2.1210   -1.5844    0.5008
   %  1.0439    1.3607   -0.8079   -2.1100    1.5673   -0.5370
   %  1.0439    0.5979    0.8079   -2.9628    1.5673   -0.5370
   %  1.0439    1.4190   -1.8038    1.9692   -1.5673    2.6046
   %  1.0439   -0.2518    1.8038    0.0325   -1.5673    2.6046
   % 
    trayectoria = [
    punto_medio 0
    aux_1 0
    caja_origen_superior 0
    caja_origen_inferior 1
    caja_origen_superior 1
    aux_1 0
    punto_medio 0
    aux_2 0
    caja_destino_superior 0
    caja_destino_inferior 1
    caja_destino_superior 1
    aux_2 0 
    punto_medio 0
    ];

    qdmax = [180 180 180 360 360 360]*d;
    qddmax = [800 800 800 800 800 800]*d;

    q_traj = gen_traj(R, trayectoria, q0, qdmax, qddmax, dt, 0);

    %% Visualización

    % grafQaE(R, q_traj);
    myAnimate(R,q_traj, 0.0001);

    % qd_traj  = zeros(size(q_traj));
    % qdd_traj = zeros(size(q_traj));
    % for j = 1:size(q_traj,2)
    %     qd_traj(:,j)  = gradient(q_traj(:,j), dt);  
    %     qdd_traj(:,j) = gradient(qd_traj(:,j), dt);  
    % end
    % 
    % grafQ(q_traj,qd_traj,qdd_traj);


end