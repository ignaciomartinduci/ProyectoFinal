function [] = test_workspace(R)

    step = 5*pi/180;


    q2 = R.qlim(2,1):step:R.qlim(2,2);
    q3 = R.qlim(3,1):step:R.qlim(3,2);

    puntos_alcanzados = zeros(numel(q2)*numel(q3),2);
    idx = 0;


    for i = 1:numel(q2)
        for j = 1:numel(q3)

            q = [0 q2(i) q3(j) 0];

              T_03 = R.base;

              for s = 1:4
                T_03 = T_03 * R.links(s).A(q(s));
              end

              p = transl(T_03);
              idx = idx + 1;
              puntos_alcanzados(idx,:) = [p(1), p(3)];

        end
    end

    figure;
    plot(puntos_alcanzados(:,1),puntos_alcanzados(:,2),"Marker","*","LineStyle","none");
    title("Comprobación gruesa del espacio de trabajo SB-S4");
    xlabel("x [m]");
    ylabel("z [m]");
    hold on;
    grid minor;
    hold off;

end