function [all_sol, q_mejor] = inv_kinematics(x_t,y_t,z_t,alpha,beta,gamma, q_previo, verbose, R)

    % Singularidades 
        % Muñeca: Ocurre cuando θ4 y θ5 son paralelos -> θ5 = 0 +-180 +-360
        % Codo: Se forma un plano por las articulaciones 2, 3 y 4 -> θ3 = 0
        % Hombro: El punto de intersección entre ejes 5 y 6 se coloca sobre el plano que pasa por las articulaciones 1 y 2.

    eps = 10^-6;

    base = [R.base.n R.base.o R.base.a R.base.t; [0 0 0 1]];
    tool = [R.tool.n R.tool.o R.tool.a R.tool.t; [0 0 0 1]];


    T_06 = transl(x_t,y_t,z_t) * trotz(gamma)*troty(beta)*trotx(alpha); % RPY ZYX
    T_06_full = T_06;
    T_06 = base \ T_06_full / tool;
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

    d_3 = R.links(3).d;
    d_5 = R.links(5).d;

    a_2 = R.links(2).a;
    a_3 = R.links(3).a;

    qlim_1_inf = R.qlim(1,1);
    qlim_1_sup = R.qlim(1,2);
    qlim_2_inf = R.qlim(2,1);
    qlim_2_sup = R.qlim(2,2);
    qlim_3_inf = R.qlim(3,1);
    qlim_3_sup = R.qlim(3,2);
    qlim_4_inf = R.qlim(4,1);
    qlim_4_sup = R.qlim(4,2);
    qlim_5_inf = R.qlim(5,1);
    qlim_5_sup = R.qlim(5,2);
    qlim_6_inf = R.qlim(6,1);
    qlim_6_sup = R.qlim(6,2);
    

    %% Inicializo variables

    all_sol = [];

    q_11 = 0;
    q_12 = 0;
    q_611 = 0;
    q_612 = 0;
    q_621 = 0;
    q_622 = 0;
    q_511 = 0;
    q_512 = 0;
    q_521 = 0;
    q_522 = 0;
    q_21 = 0;
    q_22 = 0;
    q_23 = 0;
    q_24 = 0;
    q_25 = 0;
    q_26 = 0;
    q_27 = 0;
    q_28 = 0;
    q_31 = 0;
    q_32 = 0;
    q_33 = 0;
    q_34 = 0;
    q_35 = 0;
    q_36 = 0;
    q_37 = 0;
    q_38 = 0;
    q_41 = 0;
    q_42 = 0;
    q_43 = 0;
    q_44 = 0;
    q_45 = 0;
    q_46 = 0;
    q_47 = 0;
    q_48 = 0;
    A = 0;
    B = 0;
    a = 0;
    b = 0;
    C = 0;
    D = 0;
    E = 0;
    num = 0;
    den = 0;

    s1_valid = 1;
    s2_valid = 1;
    s3_valid = 1;
    s4_valid = 1;
    s5_valid = 1;
    s6_valid = 1;
    s7_valid = 1;
    s8_valid = 1;
    
    q_mejor = q_previo;

    try

        %% 1 Determinar q1
    
        A = y;
        B = -x;
        C = -d_3;
    
        t_11 = (-2*B+sqrt(4*B^2-4*(C-A)*(C+A)))/(2*(C-A));
        t_12 = (-2*B-sqrt(4*B^2-4*(C-A)*(C+A)))/(2*(C-A));
    
        q_11 = 2*atan(t_11);
        q_12 = 2*atan(t_12);
    
        if ~isreal(q_11)
            s1_valid = 0;
            s2_valid = 0;
            s3_valid = 0;
            s4_valid = 0;
        else
            q_11 = mod(q_11-qlim_1_inf, qlim_1_sup-qlim_1_inf) + qlim_1_inf;
        end
    
        if ~isreal(q_12)
            s5_valid = 0;
            s6_valid = 0;
            s7_valid = 0;
            s8_valid = 0;
        else
             q_12 = mod(q_12-qlim_1_inf, qlim_1_sup-qlim_1_inf) + qlim_1_inf;
        end
    
        
        %% 2 Determinar q6 
    
        A = -(r_22*cos(q_11)-r_12*sin(q_11));
        B = r_21*cos(q_11)-r_11*sin(q_11);
    
        if isreal(A) && isreal(B)
            q_611 = atan2(A,B);
            q_611 = mod(q_611-qlim_6_inf, qlim_6_sup-qlim_6_inf) + qlim_6_inf;
            q_612 = mod(q_611-qlim_6_inf+pi, qlim_6_sup-qlim_6_inf) + qlim_6_inf;
    
        else
            s1_valid = 0;
            s2_valid = 0;
            s3_valid = 0;
            s4_valid = 0;
        end
    
        A = -(r_22*cos(q_12)-r_12*sin(q_12));
        B = r_21*cos(q_12)-r_11*sin(q_12);
    
        if isreal(A) && isreal(B)
            q_621 = atan2(A,B);
            q_621 = mod(q_621-qlim_6_inf, qlim_6_sup-qlim_6_inf) + qlim_6_inf;
            q_622 = mod(q_621-qlim_6_inf+pi, qlim_6_sup-qlim_6_inf) + qlim_6_inf;
    
        else
            s5_valid = 0;
            s6_valid = 0;
            s7_valid = 0;
            s8_valid = 0;
        end
    
        if ~isreal(q_611)
            s1_valid = 0;
            s2_valid = 0;
        end
        if ~isreal(q_612)
            s3_valid = 0;
            s4_valid = 0;
        end
        if ~isreal(q_621)
            s5_valid = 0;
            s6_valid = 0;
        end
        if ~isreal(q_622)
            s7_valid = 0;
            s8_valid = 0;
        end
    
        %% 3 Determinar q5
    
        num = cos(q_611)*(r_21*cos(q_11)-r_11*sin(q_11))-sin(q_611)*(r_22*cos(q_11)-r_12*sin(q_11));
        den = r_23*cos(q_11)-r_13*sin(q_11);
    
        if isreal(num) && isreal(den) && s1_valid && s2_valid
            q_511 = atan2(num,den);
            q_511 = mod(q_511-qlim_5_inf, qlim_5_sup-qlim_5_inf) + qlim_5_inf;
        else
            s1_valid = 0;
            s2_valid = 0;
        end
    
        num = cos(q_612)*(r_21*cos(q_11)-r_11*sin(q_11))-sin(q_612)*(r_22*cos(q_11)-r_12*sin(q_11));
        den = r_23*cos(q_11)-r_13*sin(q_11);
    
        if isreal(num) && isreal(den) && s3_valid && s4_valid
            q_512 = atan2(num,den);
            q_512 = mod(q_512-qlim_5_inf, qlim_5_sup-qlim_5_inf) + qlim_5_inf;
        else
            s3_valid = 0;
            s4_valid = 0;
        end
    
        num = cos(q_621)*(r_21*cos(q_12)-r_11*sin(q_12))-sin(q_621)*(r_22*cos(q_12)-r_12*sin(q_12));
        den = r_23*cos(q_12)-r_13*sin(q_12);
    
        if isreal(num) && isreal(den) && s5_valid && s6_valid
            q_521 = atan2(num,den);
            q_521 = mod(q_521-qlim_5_inf, qlim_5_sup-qlim_5_inf) + qlim_5_inf;
        else
            s5_valid = 0;
            s6_valid = 0;
        end
    
        num = cos(q_622)*(r_21*cos(q_12)-r_11*sin(q_12))-sin(q_622)*(r_22*cos(q_12)-r_12*sin(q_12));
        den = r_23*cos(q_12)-r_13*sin(q_12);
    
        if isreal(num) && isreal(den) && s7_valid && s8_valid
            q_522 = atan2(num,den);
            q_522 = mod(q_522-qlim_5_inf, qlim_5_sup-qlim_5_inf) + qlim_5_inf;
        else
            s7_valid = 0;
            s8_valid = 0;
        end
    
        if ~isreal(q_511)
            s1_valid = 0;
            s2_valid = 0;
        end
        if ~isreal(q_512)
            s3_valid = 0;
            s4_valid = 0;
        end
        if ~isreal(q_521)
            s5_valid = 0;
            s6_valid = 0;
        end
        if ~isreal(q_522)
            s7_valid = 0;
            s8_valid = 0;
        end
    
    
        %% 4 Determinar q2
    
        % sol 1 - 2
    
        if s1_valid && s2_valid
            A = (r_13*cos(q_11)+r_23*sin(q_11))/(-sin(q_511));
            try
                B = -r_33/-sin(q_511);
            catch
                s1_valid = 0;
                s2_valid = 0;
            end
        
            a = -d_5e*A+z;
            b = d_5*B-x*cos(q_11)-y*sin(q_11);  
        
            C = a_2^2+a^2+b^2-a_3^2;
            D = 2*a_2*b;
            E = -2*a_2*a;
        
            t_21 = (-2*D+sqrt(4*D^2-4*(C-E)*(C+E)))/(2*(C-E));
            t_22 = (-2*D-sqrt(4*D^2-4*(C-E)*(C+E)))/(2*(C-E));
        
            q_21 = 2*atan(t_21);
            q_22 = 2*atan(t_22);
        end
        
        % sol 3 - 4
    
        if s3_valid && s4_valid
        
            A = (r_13*cos(q_11)+r_23*sin(q_11))/(-sin(q_512));
            try
                B = -r_33/-sin(q_512);
            catch
                s3_valid = 0;
                s4_valid = 0;
            end
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_11)-y*sin(q_11);
        
            C = a_2^2+a^2+b^2-a_3^2;
            D = 2*a_2*b;
            E = -2*a_2*a;
        
            t_23 = (-2*D+sqrt(4*D^2-4*(C-E)*(C+E)))/(2*(C-E));
            t_24 = (-2*D-sqrt(4*D^2-4*(C-E)*(C+E)))/(2*(C-E));
        
            q_23 = 2*atan(t_23);
            q_24 = 2*atan(t_24);
        end
    
        % sol 5 - 6
    
        if s5_valid && s6_valid
    
            A = (r_13*cos(q_12)+r_23*sin(q_12))/(-sin(q_521));
            try
                B = -r_33/-sin(q_521);
            catch
                s5_valid = 0;
                s6_valid = 0;
            end
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_12)-y*sin(q_12);
        
            C = a_2^2+a^2+b^2-a_3^2;
            D = 2*a_2*b;
            E = -2*a_2*a;
        
            t_25 = (-2*D+sqrt(4*D^2-4*(C-E)*(C+E)))/(2*(C-E));
            t_26 = (-2*D-sqrt(4*D^2-4*(C-E)*(C+E)))/(2*(C-E));
        
            q_25 = 2*atan(t_25);
            q_26 = 2*atan(t_26);
        end
    
        % sol 7 - 8
        
        if s7_valid && s8_valid
    
            A = (r_13*cos(q_12)+r_23*sin(q_12))/(-sin(q_522));
            try
                B = -r_33/-sin(q_522);
            catch
                s7_valid = 0;
                s8_valid = 0;
            end
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_12)-y*sin(q_12);  
        
            C = a_2^2+a^2+b^2-a_3^2;
            D = 2*a_2*b;
            E = -2*a_2*a;
        
            t_27 = (-2*D+sqrt(4*D^2-4*(C-E)*(C+E)))/(2*(C-E));
            t_28 = (-2*D-sqrt(4*D^2-4*(C-E)*(C+E)))/(2*(C-E));
        
            q_27 = 2*atan(t_27);
            q_28 = 2*atan(t_28);
                
        end
    
        if ~isreal(q_21)
            s1_valid = 0;
        end
        if ~isreal(q_22)
            s2_valid = 0;
        end
        if ~isreal(q_23)
            s3_valid = 0;
        end
        if ~isreal(q_24)
            s4_valid = 0;
        end
        if ~isreal(q_25)
            s5_valid = 0;
        end
        if ~isreal(q_26)
            s6_valid = 0;
        end
        if ~isreal(q_27)
            s7_valid = 0;
        end
        if ~isreal(q_28)
            s8_valid = 0;
        end
    
        q_21 = mod(q_21-qlim_2_inf, qlim_2_sup-qlim_2_inf) + qlim_2_inf;
        q_22 = mod(q_22-qlim_2_inf, qlim_2_sup-qlim_2_inf) + qlim_2_inf;
        q_23 = mod(q_23-qlim_2_inf, qlim_2_sup-qlim_2_inf) + qlim_2_inf;
        q_24 = mod(q_24-qlim_2_inf, qlim_2_sup-qlim_2_inf) + qlim_2_inf;
        q_25 = mod(q_25-qlim_2_inf, qlim_2_sup-qlim_2_inf) + qlim_2_inf;
        q_26 = mod(q_26-qlim_2_inf, qlim_2_sup-qlim_2_inf) + qlim_2_inf;
        q_27 = mod(q_27-qlim_2_inf, qlim_2_sup-qlim_2_inf) + qlim_2_inf;
        q_28 = mod(q_28-qlim_2_inf, qlim_2_sup-qlim_2_inf) + qlim_2_inf;
    
        
        %% 5 Determinar q3
    
        % sol 1 - 2
    
        if s1_valid
    
            A = (r_13*cos(q_11)+r_23*sin(q_11))/(-sin(q_511));
            B = -r_33/-sin(q_511);
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_11)-y*sin(q_11);  
        
            num = -a_2*sin(q_21)-b;
            den = -a_2*cos(q_21)+a;
        
            q23 = atan2(num,den);
        
            q_31 = q23 - q_21;
    
        end
    
        if s2_valid
    
            A = (r_13*cos(q_11)+r_23*sin(q_11))/(-sin(q_511));
            B = -r_33/-sin(q_511);
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_11)-y*sin(q_11);  
        
            num = -a_2*sin(q_22)-b;
            den = -a_2*cos(q_22)+a;
        
            q23 = atan2(num,den);
        
            q_32 = q23 - q_22;
    
        end
    
        % sol 3 - 4
    
        if s3_valid
    
            A = (r_13*cos(q_11)+r_23*sin(q_11))/(-sin(q_512));
            B = -r_33/-sin(q_512);
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_11)-y*sin(q_11);
        
            num = -a_2*sin(q_23)-b;
            den = -a_2*cos(q_23)+a;
        
            q23 = atan2(num,den);
        
            q_33 = q23 - q_23;
    
        end
    
        if s4_valid
    
            A = (r_13*cos(q_11)+r_23*sin(q_11))/(-sin(q_512));
            B = -r_33/-sin(q_512);
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_11)-y*sin(q_11);
        
            num = -a_2*sin(q_24)-b;
            den = -a_2*cos(q_24)+a;
        
            q23 = atan2(num,den);
        
            q_34 = q23 - q_24;
    
        end
    
        % sol 5 - 6
    
        if s5_valid
    
            A = (r_13*cos(q_12)+r_23*sin(q_12))/(-sin(q_521));
            B = -r_33/-sin(q_521);
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_12)-y*sin(q_12);
        
            num = -a_2*sin(q_25)-b;
            den = -a_2*cos(q_25)+a;
        
            q23 = atan2(num,den);
        
            q_35 = q23 - q_25;
    
        end
    
        if s6_valid
    
            A = (r_13*cos(q_12)+r_23*sin(q_12))/(-sin(q_521));
            B = -r_33/-sin(q_521);
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_12)-y*sin(q_12);
        
            num = -a_2*sin(q_26)-b;
            den = -a_2*cos(q_26)+a;
        
            q23 = atan2(num,den);
        
            q_36 = q23 - q_26;
    
        end
    
        % sol 7 - 8
    
        if s7_valid
        
            A = (r_13*cos(q_12)+r_23*sin(q_12))/(-sin(q_522));
            B = -r_33/-sin(q_522);
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_12)-y*sin(q_12);  
        
            num = -a_2*sin(q_27)-b;
            den = -a_2*cos(q_27)+a;
        
            q23 = atan2(num,den);
        
            q_37 = q23 - q_27;
    
        end
    
        if s8_valid
        
            A = (r_13*cos(q_12)+r_23*sin(q_12))/(-sin(q_522));
            B = -r_33/-sin(q_522);
        
            a = -d_5*A+z;
            b = d_5*B-x*cos(q_12)-y*sin(q_12);
        
            num = -a_2*sin(q_28)-b;
            den = -a_2*cos(q_28)+a;
        
            q23 = atan2(num,den);
        
            q_38 = q23 - q_28;
    
        end
    
        q_31 = mod(q_31-qlim_3_inf, qlim_3_sup-qlim_3_inf) + qlim_3_inf;
        q_32 = mod(q_32-qlim_3_inf, qlim_3_sup-qlim_3_inf) + qlim_3_inf;
        q_33 = mod(q_33-qlim_3_inf, qlim_3_sup-qlim_3_inf) + qlim_3_inf;
        q_34 = mod(q_34-qlim_3_inf, qlim_3_sup-qlim_3_inf) + qlim_3_inf;
        q_35 = mod(q_35-qlim_3_inf, qlim_3_sup-qlim_3_inf) + qlim_3_inf;
        q_36 = mod(q_36-qlim_3_inf, qlim_3_sup-qlim_3_inf) + qlim_3_inf;
        q_37 = mod(q_37-qlim_3_inf, qlim_3_sup-qlim_3_inf) + qlim_3_inf;
        q_38 = mod(q_38-qlim_3_inf, qlim_3_sup-qlim_3_inf) + qlim_3_inf;
    
    
        %% 6 Determinar q4
    
        % Sol 1
    
            num = r_33;
        den = -(r_13*cos(q_11)+r_23*sin(q_11));
    
        if s1_valid
            q234 = atan2(num,den);
            q_41 = q234 - q_21 - q_31;
        end
    
        % Sol 2
    
        num = r_33;
        den = -(r_13*cos(q_11)+r_23*sin(q_11));
    
        if s2_valid
            q234 = atan2(num,den);
            q_42 = q234 - q_22 - q_32;
        end
    
        % Sol 3
    
        num = r_33;
        den = -(r_13*cos(q_11)+r_23*sin(q_11));
    
        if s3_valid
            q234 = atan2(num,den);
            q_43 = (q234 - q_23 - q_33)+pi;
        end
    
    
        % Sol 4    
    
        num = r_33;
        den = -(r_13*cos(q_11)+r_23*sin(q_11));
    
        if s4_valid
            q234 = atan2(num,den);
            q_44 = q234 - q_24 - q_34+pi;
        end
    
    
        % Sol 5
    
        num = r_33;
        den = -(r_13*cos(q_12)+r_23*sin(q_12));
    
        if s5_valid
            q234 = atan2(num,den);
            q_45 = q234 - q_25 - q_35;
        end
    
        % Sol 6
    
        num = r_33;
        den = -(r_13*cos(q_12)+r_23*sin(q_12));
    
        if s6_valid
            q234 = atan2(num,den);
            q_46 = q234 - q_26 - q_36;
        end
    
        % Sol 7
    
        num = r_33;
        den = -(r_13*cos(q_12)+r_23*sin(q_12));
    
        if s7_valid
            q234 = atan2(num,den);
            q_47 = q234 - q_27 - q_37+pi;
        end
    
        % Sol 8
    
        num = r_33;
        den = -(r_13*cos(q_12)+r_23*sin(q_12));
    
        if s8_valid
            q234 = atan2(num,den);
            q_48 = q234 - q_28 - q_38+pi;
        end
    
        q_41 = mod(q_41-qlim_4_inf, qlim_4_sup-qlim_4_inf) + qlim_4_inf;
        q_42 = mod(q_42-qlim_4_inf, qlim_4_sup-qlim_4_inf) + qlim_4_inf;
        q_43 = mod(q_43-qlim_4_inf, qlim_4_sup-qlim_4_inf) + qlim_4_inf;
        q_44 = mod(q_44-qlim_4_inf, qlim_4_sup-qlim_4_inf) + qlim_4_inf;
        q_45 = mod(q_45-qlim_4_inf, qlim_4_sup-qlim_4_inf) + qlim_4_inf;
        q_46 = mod(q_46-qlim_4_inf, qlim_4_sup-qlim_4_inf) + qlim_4_inf;
        q_47 = mod(q_47-qlim_4_inf, qlim_4_sup-qlim_4_inf) + qlim_4_inf;
        q_48 = mod(q_48-qlim_4_inf, qlim_4_sup-qlim_4_inf) + qlim_4_inf;

        all_sol = [];

        if s1_valid
            all_sol = [q_11 q_21 q_31 q_41 q_511 q_611];
            if verbose
                disp("S1 valida")
            end
        end
    
        if s2_valid
            all_sol = [all_sol; q_11 q_22 q_32 q_42 q_511 q_611];
            if verbose
                disp("S2 valida")
            end
        end
    
        if s3_valid
            all_sol = [all_sol; q_11 q_23 q_33 q_43 q_512 q_612];
            if verbose
                disp("S3 valida")
            end
        end
    
        if s4_valid
            all_sol = [all_sol; q_11 q_24 q_34 q_44 q_512 q_612];
            if verbose
                disp("S4 valida")
            end
        end
    
        if s5_valid
            all_sol = [all_sol; q_12 q_25 q_35 q_45 q_521 q_621];
            if verbose
                disp("S5 valida")
            end
        end
    
        if s6_valid
            all_sol = [all_sol; q_12 q_26 q_36 q_46 q_521 q_621];
            if verbose
                disp("S6 valida")
            end
        end
    
        if s7_valid
            all_sol = [all_sol; q_12 q_27 q_37 q_47 q_522 q_622];
            if verbose
                disp("S7 valida")
            end
        end
    
        if s8_valid
            all_sol = [all_sol; q_12 q_28 q_38 q_48 q_522 q_622];
            if verbose
                disp("S8 valida")
            end
        end 


    catch
        if verbose
            disp("Error en el cálculo de soluciones IK analítico")
        end
    end
    

    %% ALTERNATIVA NUMÉRICA

    
    if isempty(all_sol)
        if verbose
            disp("No se encontraron soluciones mediante método algebráico, probando método numérico.")
        end
        
        try 
            q_numerica = R.ikine(T_06_full, q_previo, 'tol', 1e-2, 'ilimit', 4000, 'pinv_daming',1e-3);
            all_sol = q_numerica;
        catch
            disp("No se encontraron soluciones numéricas")
        end
        if isempty(all_sol)
            if verbose
                disp("No se encontraron soluciones numéricas")
            end
        end
    
    end


    

end

