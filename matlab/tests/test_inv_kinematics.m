function [] = test_inv_kinematics(R ,TEST_LOOP_IK, PRINT_SOLUTIONS)
disp(' ');
disp("---> TEST INV_KINEMATICS")

d = pi/180;
e = 0.01;
q_tik = [1.8358    0.4922   -1.9151   -0.5557    1.7815    1.2078];
%R.plot(q_tik)

q_previo = [0 0 0 0 0 0]*d;
disp("Posición articular previa")
disp(q_previo)

disp("Probando posición articular: ");
disp(q_tik)
T_tik = R.fkine(q_tik);

p_tik= transl(T_tik);
rpy_tik = tr2rpy(T_tik,'zyx');

x_tik = p_tik(1);
y_tik = p_tik(2);
z_tik = p_tik(3);

alpha_tik = rpy_tik(1);
beta_tik = rpy_tik(2);
gamma_tik = rpy_tik(3);

% override
x_tik = -0.202;
y_tik = 0.333;
z_tik = 0.205;
% %
alpha_tik = -3.132;
beta_tik = -0.01;
gamma_tik = -3.132;

coordenadas_cartesianas = "x = "+string(x_tik)+" | y = "+string(y_tik)+" | z = "+string(z_tik)+" | alpha = "+string(alpha_tik)+" | beta = "+string(beta_tik)+" | gamma = "+string(gamma_tik);

disp("Las coordenadas cartesianas consigna son: ")
disp(coordenadas_cartesianas)


[sol_tik, q_mejor] = inv_kinematics(x_tik,y_tik,z_tik,alpha_tik,beta_tik,gamma_tik, q_previo, 1, R, "FORCE");

if ~isempty(sol_tik)
    
    disp("Las coordenadas articulares solución obtenidas de la cinematica inversa son:")
    disp(sol_tik);
    disp("Las coordenadas cartesianas solución obtenidas son las siguientes")
    
    for i=1:length(sol_tik(:,1))
        
        T_tik_sol = R.fkine(sol_tik(i,:));
        
        p_tik_sol = transl(T_tik_sol);
        rpy_tik_sol = tr2rpy(T_tik_sol,'zyx');
        
        x_tik_sol = p_tik_sol(1);
        y_tik_sol = p_tik_sol(2);
        z_tik_sol = p_tik_sol(3);
        
        alpha_tik_sol = rpy_tik_sol(1);
        beta_tik_sol = rpy_tik_sol(2);
        gamma_tik_sol = rpy_tik_sol(3);
        
        coordenadas_cartesianas = "x = "+string(x_tik_sol)+" | y = "+string(y_tik_sol)+" | z = "+string(z_tik_sol)+" | alpha = "+string(alpha_tik_sol)+" | beta = "+string(beta_tik_sol)+" | gamma = "+string(gamma_tik_sol);
        
        disp(coordenadas_cartesianas)
        
    end
else
    
    disp("No se han encontrado soluciones válidas");
end

disp("La mejor solución es")
disp(q_mejor)

%% IMPRESIÓN DE SOLUCIONES

% solutions(R, sol_tik);

%% RESUMEN ESTADÍSTICO

disp(" ")
disp("--- Resumen estadístico IK (poses aleatorias) ---")

N_batch   = 300;
err_tol   = 1e-3;   % [m] umbral para considerar solución válida
n_ok      = 0;
n_sin_sol = 0;
err_pos_v = [];
err_ang_v = [];
q_ref     = zeros(1,6);

for k = 1:N_batch
    q_rand = zeros(1,6);
    for j = 1:6
        q_rand(j) = R.qlim(j,1) + (R.qlim(j,2) - R.qlim(j,1)) * rand;
    end

    T_tgt = double(R.fkine(q_rand));
    rpy   = tr2rpy(T_tgt, 'zyx');

    [ampl_sol, q_mejor] = inv_kinematics(T_tgt(1,4), T_tgt(2,4), T_tgt(3,4), ...
        rpy(1), rpy(2), rpy(3), q_ref, false, R, "NONE");

    if size(ampl_sol, 1) >= 64
        T_check = double(R.fkine(q_mejor));
        ep = norm(T_check(1:3,4) - T_tgt(1:3,4));
        ea = acos(min(1, max(-1, (trace(T_check(1:3,1:3)' * T_tgt(1:3,1:3)) - 1) / 2)));
        if ep < err_tol
            n_ok = n_ok + 1;
            err_pos_v(end+1) = ep; %#ok<AGROW>
            err_ang_v(end+1) = ea; %#ok<AGROW>
        end
    else
        n_sin_sol = n_sin_sol + 1;
    end
end

fprintf("  Poses testeadas:    %d\n",           N_batch)
fprintf("  Sin solución IK:    %d\n",           n_sin_sol)
fprintf("  Poses verificadas:  %d  (%.1f%%)\n", n_ok, 100*n_ok/N_batch)
if ~isempty(err_pos_v)
    fprintf("  Error posición    — max: %.2e m    prom: %.2e m\n",   max(err_pos_v), mean(err_pos_v))
    fprintf("  Error orientación — max: %.2e rad  prom: %.2e rad\n", max(err_ang_v), mean(err_ang_v))
end

%% LOOP TESTING

if TEST_LOOP_IK
    
    disp(" ")
    disp(" ")
    disp(" ")
    disp("LOOP -> Comenzando prueba en loop")
    
    while 1
        
        q1 =  R.qlim(1,1) + (R.qlim(1,2)-R.qlim(1,1))*rand;
        q2 =  R.qlim(2,1) + (R.qlim(2,2)-R.qlim(2,1))*rand;
        q3 =  R.qlim(3,1) + (R.qlim(3,2)-R.qlim(3,1))*rand;
        q4 =  R.qlim(4,1) + (R.qlim(4,2)-R.qlim(4,1))*rand;
        q5 =  R.qlim(5,1) + (R.qlim(5,2)-R.qlim(5,1))*rand;
        q6 =  -2*pi + (2*pi-2*pi)*rand;
        
        q_loop = [q1 q2 q3 q4 q5 q6];
        disp("LOOP -> Probando posición articular");
        disp(q_loop);
        
        T_loop = R.fkine(q_loop);
        T_loop = [T_loop.n T_loop.o T_loop.a T_loop.t ; 0 0 0 1];
        
        p_loop= transl(T_loop);
        rpy_loop = tr2rpy(T_loop,'zyx');
        
        x_loop = p_loop(1);
        y_loop = p_loop(2);
        z_loop = p_loop(3);
        
        alpha_loop = rpy_loop(1);
        beta_loop = rpy_loop(2);
        gamma_loop = rpy_loop(3);
        
        coordenadas_cartesianas = "x = "+string(x_loop)+" | y = "+string(y_loop)+" | z = "+string(z_loop)+" | alpha = "+string(alpha_loop)+" | beta = "+string(beta_loop)+" | gamma = "+string(gamma_loop);
        
        disp("LOOP -> Las coordenadas cartesianas consigna son: ")
        disp(coordenadas_cartesianas)
        
        [sol_loop, ~] = inv_kinematics(x_loop,y_loop,z_loop,alpha_loop,beta_loop,gamma_loop, q_previo, 1, R)
        
        if ~isempty(sol_loop)
            for i=1:length(sol_loop(:,1))
                
                T_sol = R.fkine(sol_loop(i,:));
                T_sol = [T_sol.n T_sol.o T_sol  .a T_sol.t ; 0 0 0 1];
                
                dp = norm(transl(T_loop) - transl(T_sol));
                dang = acos((trace(T_loop(1:3,1:3)'*T_sol(1:3,1:3)) - 1)/2);   % diferencia angular
                
                
                if dp >= 10^-2 || dang >= 10^-2
                    disp("LOOP -> Posible error de IK - detenido para verificar")
                    p_sol = transl(T_sol);
                    rpy_sol = tr2rpy(T_sol,'zyx');
                    
                    x_sol = p_sol(1);
                    y_sol = p_sol(2);
                    z_sol = p_sol(3);
                    
                    alpha_sol = rpy_sol(1);
                    beta_sol = rpy_sol(2);
                    gamma_sol = rpy_sol(3);
                    coordenadas_cartesianas = "x = "+string(x_sol)+" | y = "+string(y_sol)+" | z = "+string(z_sol)+" | alpha = "+string(alpha_sol)+" | beta = "+string(beta_sol)+" | gamma = "+string(gamma_sol);
                    
                    disp("LOOP -> Las coordenadas cartesianas resultantes son: ")
                    disp(coordenadas_cartesianas)
                    input("")
                else
                    disp("OK")
                end
                
            end
        end
        
        pause(0.5);
        
        
    end
    
end

end