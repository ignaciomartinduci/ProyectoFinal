function [all_sol, q_mejor] = inv_kinematics(x_t,y_t,z_t,alpha,beta,gamma, d_4, d_5, a_2, a_3, base, tool, q_previo, verbose, R)

    % Singularidades 
        % Muñeca: Ocurre cuando θ4 y θ5 son paralelos -> θ5 = 0 +-180 +-360
        % Codo: Se forma un plano por las articulaciones 2, 3 y 4 -> θ3 = 0
        % Hombro: El punto de intersección entre ejes 5 y 6 se coloca sobre el plano que pasa por las articulaciones 1 y 2.

    T_06 = transl(x_t,y_t,z_t) * trotz(gamma)*troty(beta)*trotx(alpha); % RPY ZYX
    T_06 = inv(base)*T_06*inv(tool);
    r_11 = T_06(1,1);
    r_12 = T_06(1,2);
    r_13 = T_06(1,3);
    r_21 = T_06(2,1);
    r_22 = T_06(2,2);
    r_23 = T_06(2,3);
    r_31 = T_06(3,1);
    r_32 = T_06(3,2);
    r_33 = T_06(3,3);

    x = T_06(1,4);
    y = T_06(2,4);
    z = T_06(3,4);

    %% Inicializo variables
    q_11 = 0;
    q_12 = 0;
    q_61 = 0;
    q_62 = 0;
    q_51 = 0;
    q_52 = 0;
    q_211 = 0;
    q_212 = 0;
    q_221 = 0;
    q_222 = 0;
    q_311 = 0;
    q_312 = 0;
    q_321 = 0;
    q_322 = 0;
    q_411 = 0;
    q_412 = 0;
    q_421 = 0;
    q_422 = 0;
    A = 0;
    B = 0;
    a = 0;
    b = 0;
    num = 0;
    den = 0;
    

    %% 1 Determinar q1

    % t_11 = (x+sqrt(x^2-(d_4-y)*(d_4+y)))/(d_4-y);
    % t_12 = (x-sqrt(x^2-(d_4-y)*(d_4+y)))/(d_4-y); 
    % 
    % q_11 = 2*atan(t_11);
    % q_12 = 2*atan(t_12);

    %% 2 Determinar q6

    % if isreal(q_11)
    %     q_61 = atan((r_12*sin(q_11)-r_22*cos(q_11)) / (r_21*cos(q_11)-r_11*sin(q_11)));
    % end
    % 
    % if isreal(q_12)
    %     q_62 = atan((r_12*sin(q_12)-r_22*cos(q_12)) / (r_21*cos(q_12)-r_11*sin(q_12)));
    % end

    %% 3 Determinar q5

    % if isreal(q_61)&&isreal(q_11)
    %     num = (r_21*cos(q_11)-r_11*sin(q_11))*cos(q_61)-(r_22*cos(q_11)-r_12*sin(q_11))*sin(q_61);
    %     den = -r_23*cos(q_11) + r_13*sin(q_11);
    % 
    %     q_51 = atan(num/den);
    % end
    % 
    % if isreal(q_62)&&isreal(q_12)
    %     num = (r_21*cos(q_12)-r_11*sin(q_12))*cos(q_62)-(r_22*cos(q_12)-r_12*sin(q_12))*sin(q_62);
    %     den = -r_23*cos(q_12) + r_13*sin(q_12);
    % 
    %     q_52 = atan(num/den);
    % end

    %% 4 Determinar q2

    % if isreal(q_61)&&isreal(q_11)
    %     A = (r_31*cos(q_61)-r_32*sin(q_61))/cos(q_51);
    %     B = r_32*cos(q_61)+r_31*sin(q_61);
    % 
    %     a = -x*cos(q_11)-y*sin(q_11)-d_5*A;
    %     b = z-d_5*B;
    % 
    %     E = -2*a_2*b;
    %     F = -2*a_2*a;
    %     G = a_2^2+a^2+b^2-a_3^2;
    % 
    %     t_211 = (-F+sqrt(E^2+F^2-G^2))/(G-E);
    %     t_212 = (-F-sqrt(E^2+F^2-G^2))/(G-E);
    % 
    %     q_211 = 2*atan(t_211);
    %     q_212 = 2*atan(t_212);
    % end
    % 
    % if isreal(q_62)&&isreal(q_12)
    %     A = (r_31*cos(q_62)-r_32*sin(q_62))/cos(q_52);
    %     B = r_32*cos(q_62)+r_31*sin(q_62);
    % 
    %     a = -x*cos(q_12)-y*sin(q_12)-d_5*A;
    %     b = z-d_5*B;
    % 
    %     E = -2*a_2*b;
    %     F = -2*a_2*a;
    %     G = a_2^2+a^2+b^2-a_3^2;
    % 
    %     t_221 = (-F+sqrt(E^2+F^2-G^2))/(G-E);
    %     t_222 = (-F-sqrt(E^2+F^2-G^2))/(G-E);
    % 
    %     q_221 = 2*atan(t_221);
    %     q_222 = 2*atan(t_222);
    % end

    %% 5 determinar q3

    % if isreal(q_61) && isreal(q_11)
    %     A = (r_31*cos(q_61)-r_32*sin(q_61))/cos(q_51);
    %     B = r_32*cos(q_61)+r_31*sin(q_61);
    % 
    %     a = -x*cos(q_11)-y*sin(q_11)-d_5*A;
    %     b = z-d_5*B;
    % 
    %     if isreal(q_211)
    %         num = a-a_2*sin(q_211);
    %         den = b-a_2*cos(q_211);
    % 
    %         q_311 = atan2(num,den) - q_211;
    % 
    %     end
    % 
    %     if isreal(q_212)
    %         num = a-a_2*sin(q_212);
    %         den = b-a_2*cos(q_212);
    % 
    %         q_312 = atan2(num,den) - q_212;
    %     end
    % 
    % end
    % 
    % if isreal(q_62)&&isreal(q_12)
    % 
    %     A = (r_31*cos(q_62)-r_32*sin(q_62))/cos(q_52);
    %     B = r_32*cos(q_62)+r_31*sin(q_62);
    % 
    %     a = -x*cos(q_12)-y*sin(q_12)-d_5*A;
    %     b = z-d_5*B;
    % 
    %     if isreal(q_221)
    %         num = a-a_2*sin(q_221);
    %         den = b-a_2*cos(q_221);
    % 
    %         q_321 = atan2(num,den) - q_221;
    %     end
    % 
    %     if isreal(q_222)
    %         num = a-a_2*sin(q_222);
    %         den = b-a_2*cos(q_222);
    % 
    %         q_322 = atan2(num,den) - q_222;
    %     end
    % end

    %% 6 Determinar q4

    % if isreal(q_61)&&isreal(q_51)
    %     A = (r_31*cos(q_61)-r_32*sin(q_61))/cos(q_51);
    %     B = r_32*cos(q_61)+r_31*sin(q_61);
    % 
    %     q_411 = atan2(A,B)-q_211-q_311;
    % 
    %     q_412 = atan2(A,B)-q_212-q_312;
    % end
    % 
    % if isreal(q_62)&&isreal(q_52)
    %     A = (r_31*cos(q_62)-r_32*sin(q_62))/cos(q_52);
    %     B = r_32*cos(q_62)+r_31*sin(q_62);
    % 
    %     q_421 = atan2(A,B)-q_221-q_321;
    % 
    %     q_422 = atan2(A,B)-q_222-q_322;
    % end
    % 
    % %% Formación de soluciones
    % 
    % s_1 = [q_11 q_211 q_311 q_411 q_51 q_61];
    % 
    % s_2 = [q_11 q_212 q_312 q_412 q_51 q_61];
    % 
    % s_3 = [q_12 q_221 q_321 q_421 q_52 q_62];
    % 
    % s_4 = [q_12 q_222 q_322 q_422 q_52 q_62];

    %% Descarte de soluciones

%     all_sol = [];
%     costos = [inf inf inf inf];
% 
%     s_1_valid = 0;
%     s_2_valid = 0;
%     s_3_valid = 0;
%     s_4_valid = 0;
% 
%     if isreal(s_1)
%         ok_1 = validar(R, s_1, x_t, y_t, z_t, alpha, beta, gamma);
%         if ok_1
%             s_1_valid = 1;
%             all_sol = s_1;
%             if verbose, disp("S1"); end
%             costos(1) = cost_f(s_1, q_previo);
%         end
%     end
%     if isreal(s_2)
%         ok_2 = validar(R, s_2, x_t, y_t, z_t, alpha, beta, gamma);
%         if ok_2
%             s_2_valid = 1;
%             all_sol = [all_sol; s_2];
%             if verbose, disp("S2"); end
%             costos(2) = cost_f(s_2, q_previo);
%         end
%     end
%     if isreal(s_3)
%         ok_3 = validar(R, s_3, x_t, y_t, z_t, alpha, beta, gamma);  
%         if ok_3
%                     s_3_valid = 1;
%             all_sol = [all_sol; s_3];
%             if verbose, disp("S3"); end
%             costos(3) = cost_f(s_3, q_previo);
%         end
%     end
%     if isreal(s_4)
%         ok_4 = validar(R, s_4, x_t, y_t, z_t, alpha, beta, gamma);
%         if ok_4 
%                     s_4_valid = 1;
%             all_sol = [all_sol; s_4];
%             if verbose, disp("S4"); end
%             costos(4) = cost_f(s_4, q_previo);
%         end
%     end
% 
%     valid_solutions = s_1_valid + s_2_valid + s_3_valid + s_4_valid;
% 
%     if ~isempty(all_sol)
%         for i=1:length(all_sol(:,1))
% 
%             for j=1:6
%                 while all_sol(i,j) >= 2*pi
%                     all_sol(i,j) = all_sol(i,j) - 2*pi;
% 
%                 end
%                 while all_sol(i,j) <= -2*pi
%                     all_sol(i,j) = all_sol(i,j) + 2*pi;
%                 end 
% 
%             end
%         end
%     end
% 
% 
% %% Selección de mejor respuesta
% 
%     q_mejor = q_previo;
% 
%     [cmin, idx] = min(costos);  
% 
%     if ~isempty(all_sol)
%         for i=1:length(all_sol(:,1))
% 
%             if costos(i) > 9999
%                 idx = idx -1;
%             end
%         end
%     end
% 
%     if cmin < inf 
%         q_mejor = all_sol(idx, :);
%     end
% 
%     if verbose
%         disp(" ")
%         respuesta = "Se han encontrado "+string(valid_solutions)+" soluciones validas";
%         disp(respuesta)
% 
%     end
% 
%     if isempty(all_sol)
% 
%         disp("NO SE ENCONTRARON SOLUCIONES.")
% 
%     end

% end
% 
% function costo = cost_f(q, q_previo)
%     costo = 0;
%     for i=1:length(q)
%         costo = costo + abs(q(i) - q_previo(i)); % costo por distancia entre posiciones articulares
% 
%     end
% 
% end

% function ok = validar(R, s, x_0, y_0, z_0, alpha_0, beta_0, gamma_0)
% 
%     T = R.fkine(s);
%     T = [T.n T.o T.a T.t; [0 0 0 1]];
%     rpy = tr2rpy(T,'zyx');
% 
%     x_1 = T(1,4);
%     y_1 = T(2,4);
%     z_1 = T(3,4);
%     alpha_1 = rpy(1);
%     beta_1 = rpy(2);
%     gamma_1 = rpy(3);
% 
%     tol = 0.1;
%     e = abs(x_0-x_1) + abs(y_0-y_1) + abs(z_0-z_1);
% 
%     if e <= tol
%         ok = 1;
%         return
%     else
%         ok = 0;
%         return
%     end




end