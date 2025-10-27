function [] = singularities(R, PLOT_SINGULARITIES)


    rand_num_1 = R.qlim(1,1) + (R.qlim(1,2)-R.qlim(1,1)) * rand;
    rand_num_2 = R.qlim(2,1) + (R.qlim(2,2)-R.qlim(2,1)) * rand;
    rand_num_3 = R.qlim(3,1) + (R.qlim(3,2)-R.qlim(3,1)) * rand;
    rand_num_4 = R.qlim(4,1) + (R.qlim(4,2)-R.qlim(4,1)) * rand;
    rand_num_5 = R.qlim(5,1) + (R.qlim(5,2)-R.qlim(5,1)) * rand;
    rand_num_6 = R.qlim(6,1) + (R.qlim(6,2)-R.qlim(6,1)) * rand;

    q_wrist_1 = [rand_num_1 rand_num_2 rand_num_3 rand_num_4 0 rand_num_6];
    q_wrist_2 = [rand_num_1 rand_num_2 rand_num_3 rand_num_4 pi rand_num_6];
    q_wrist_3 = [rand_num_1 rand_num_2 rand_num_3 rand_num_4 -pi rand_num_6];
    q_wrist_4 = [rand_num_1 rand_num_2 rand_num_3 rand_num_4 2*pi rand_num_6];
    q_wrist_5 = [rand_num_1 rand_num_2 rand_num_3 rand_num_4 -2*pi rand_num_6];

    q_elbow = [rand_num_1 rand_num_2 0 rand_num_4 rand_num_5 rand_num_6];

    % Falta q_shoulder
    
    all_singularities = [q_wrist_1; q_wrist_2; q_wrist_3; q_wrist_4; q_wrist_5; q_elbow];

    for i=1:length(all_singularities(:,1))
    
        J = R.jacob0(all_singularities(i,:));
        str = "Determinante del Jacobiano para la configuración "+string(i);
        disp(str);
        disp(det(J));

        if PLOT_SINGULARITIES
            R.plot(all_singularities(i,:))
            input("Detenido, enter para ver siguiente singularidad ...")
        end
    end

end