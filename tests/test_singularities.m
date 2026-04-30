function [] = singularities(R)

while 1
    
    %% Singularidad de muñeca q5 = 0
    
    q1_rand =  R.qlim(1,1) + (R.qlim(1,2)-R.qlim(1,1))*rand;
    q2_rand =  R.qlim(2,1) + (R.qlim(2,2)-R.qlim(2,1))*rand;
    q3_rand =  R.qlim(3,1) + (R.qlim(3,2)-R.qlim(3,1))*rand;
    q4_rand =  R.qlim(4,1) + (R.qlim(4,2)-R.qlim(4,1))*rand;
    q5_rand =  randi([-2, 2]);
    q6_rand =  R.qlim(6,1) + (R.qlim(6,2)-R.qlim(6,1))*rand;
    
    q_wrist = [q1_rand q2_rand q3_rand q4_rand q5_rand*pi q6_rand];
    
    J = R.jacob0(q_wrist);
    det_J = det(J);
    
    if abs(det_J) > 10^-3
        disp("Posición articular aleatoria - no singular - muñeca");
        disp(q_wrist);
        disp("Determinante del Jacobiano: "+string(det_J));
        disp("-> Detención por posición no singular - enter para continuar")
        input("");
    end
    
    %% Singularidad de codo q3 = k*pi
    
    q1_rand =  R.qlim(1,1) + (R.qlim(1,2)-R.qlim(1,1))*rand;
    q2_rand =  R.qlim(2,1) + (R.qlim(2,2)-R.qlim(2,1))*rand;
    q3_rand =  randi([-2, 2]);
    q4_rand =  R.qlim(4,1) + (R.qlim(4,2)-R.qlim(4,1))*rand;
    q5_rand =  R.qlim(5,1) + (R.qlim(5,2)-R.qlim(5,1))*rand;
    q6_rand =  R.qlim(6,1) + (R.qlim(6,2)-R.qlim(6,1))*rand;
    
    q_elbow = [q1_rand q2_rand q3_rand*pi q4_rand q5_rand q6_rand];
    
    J = R.jacob0(q_elbow);
    det_J = det(J);
    
    if abs(det_J) > 10^-3
        disp("Posición articular aleatoria - no singular - codo");
        disp(q_elbow);
        disp("Determinante del Jacobiano: "+string(det_J));
        disp("-> Detención por posición no singular - enter para continuar")
        input("");
    end
    
    
    
    %% Singularidad de hombro, S5 o S6 sobre plano definido por A1 y A2
    
    a2 = R.links(2).a;
    a3 = R.links(3).a;
    d5 = R.links(5).d;
    
    q1 = -pi + (2*pi)*rand;
    q2 = deg2rad(-10 + 20*rand);
    if q2 >= 0
        q3 = deg2rad(-10 * rand);
    else
        q3 = deg2rad(10 * rand);
    end
    q4 = -pi + (2*pi)*rand;
    q5 = -pi + (2*pi)*rand;
    q6 = -pi + (2*pi)*rand;
    
    q4 = asin((-a2*sin(q2)-a3*sin(q2+q3))/(d5))-q2-q3;
    
    
    % 4. Armar el vector y evaluar
    q_shoulder = [q1 q2 q3 q4 q5 q6];
    J = R.jacob0(q_shoulder);
    det_J = det(J);
    
    if abs(det_J) > 10^-3
        disp("Posición articular aleatoria - no singular - hombro");
        disp(q_shoulder);
        disp("Determinante del Jacobiano: "+string(det_J));
        disp("-> Detención por posición no singular - enter para continuar")
        input("");
    end
    
    disp("Ciclo completo - todas las posiciones fueron singulares - continuando...")
    
end