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

    q0 = [0    1.9977   -1.9700   -1.6126    1.5698   pi/2]; 
    
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

    q_traj = gen_traj(R, trayectoria, q0, qdmax, qddmax, dt);

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