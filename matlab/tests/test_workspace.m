    function [] = test_workspace(R)

d = pi/180;
e = 0.01;

%% Puntos de tarea (idénticos a tarea.m) a verificar dentro del espacio de trabajo

caja_origen_superior =      [-0.22, 0.3,    0.2     0*d+e 180*d+e 0*d+e];
caja_origen_inferior =      [-0.22,  0.3,    0.01    0*d+e 180*d+e 0*d+e];
aux_1 =                     [-0.1,  0.3,    0.22    0*d+e 180*d+e 0*d+e];
punto_medio =               [0,     0.3,   0.25    0*d+e 180*d+e 0*d+e];
aux_2 =                     [0.1,  0.3,    0.22    0*d+e 180*d+e 0*d+e];
caja_destino_superior =     [0.22,  0.3,    0.2     0*d+e 180*d+e 0*d+e];
caja_destino_inferior =     [0.22,  0.3,    0.01    0*d+e 180*d+e 0*d+e];

puntos_tarea = [
    caja_origen_superior
    caja_origen_inferior
    aux_1
    punto_medio
    aux_2
    caja_destino_superior
    caja_destino_inferior
    ];

%% Cajas de tarea (vista superior), definidas como rangos en x e y

caja1_x = [0.12, 0.32];
caja1_y = [0.075, 0.30];

caja2_x = [-0.32, -0.12];
caja2_y = [0.075, 0.30];

%% Vista superior: barrido punto a punto de las cajas de tarea con
%  orientación de tarea (0º 180º 0º) fija y paso de 0.01 m en x e y,
%  verificando alcanzabilidad mediante la IK

paso_verif = 0.01;
z_verif = 0;

cajas_x = {caja1_x, caja2_x};
cajas_y = {caja1_y, caja2_y};

r_base   = 0.075;
altura_caja = 0.15;
z_lat_lim   = [0, 0.25];

%% Vistas superiores: z = 0, 0.15, 0.25

z_vals = [0, 0.15, 0.25];

for zi = 1:numel(z_vals)

    z_verif = z_vals(zi);
    puntos_alcanzables    = [];
    puntos_no_alcanzables = [];

    for c = 1:numel(cajas_x)

        x_range = cajas_x{c}(1):paso_verif:cajas_x{c}(2);
        y_range = cajas_y{c}(1):paso_verif:cajas_y{c}(2);

        for i = 1:numel(x_range)
            for j = 1:numel(y_range)

                it_count   = 0;
                alpha_rand = 0;
                beta_rand  = 0;
                gamma_rand = 0;

                while true
                    [ampl_sol, ~] = inv_kinematics(x_range(i), y_range(j), z_verif, ...
                        0*d+e+alpha_rand, 180*d+e+beta_rand, 0*d+e+gamma_rand, zeros(1,6), false, R, "NONE");

                    if size(ampl_sol,1) > 1
                        break;
                    else
                        it_count = it_count + 1;
                        alpha_rand = rand()*2*pi;
                        beta_rand  = rand()*2*pi;
                        gamma_rand = rand()*2*pi;

                        if it_count > 20
                            break;
                        end
                    end
                end

                if size(ampl_sol,1) > 1
                    puntos_alcanzables    = [puntos_alcanzables;    x_range(i) y_range(j)]; %#ok<AGROW>
                else
                    puntos_no_alcanzables = [puntos_no_alcanzables; x_range(i) y_range(j)]; %#ok<AGROW>
                end

            end
        end

    end

    figure;
    hold on;
    leyendas = strings(0);

    if ~isempty(puntos_alcanzables)
        plot(puntos_alcanzables(:,1), puntos_alcanzables(:,2), ...
            "Marker",".","MarkerSize",10,"LineStyle","none","Color",[0 0.6 0]);
        leyendas(end+1) = "Puntos alcanzables";
    end
    if ~isempty(puntos_no_alcanzables)
        plot(puntos_no_alcanzables(:,1), puntos_no_alcanzables(:,2), ...
            "Marker","x","MarkerSize",6,"LineStyle","none","Color","r");
        leyendas(end+1) = "Puntos no alcanzables";
    end

    rectangle("Position",[caja1_x(1), caja1_y(1), diff(caja1_x), diff(caja1_y)], ...
        "EdgeColor","k","LineWidth",1.5);
    leyendas(end+1) = "Cajas de tarea";
    rectangle("Position",[caja2_x(1), caja2_y(1), diff(caja2_x), diff(caja2_y)], ...
        "EdgeColor","k","LineWidth",1.5,"HandleVisibility","off");

    rectangle("Position",[-r_base, -r_base, 2*r_base, 2*r_base], ...
        "Curvature",[1 1], "EdgeColor",[0.6 0.8 1], "FaceColor",[0.8 0.95 1], "LineWidth",1.5);
    leyendas(end+1) = "Base del robot";

    title("Espacio de trabajo - Vista Superior - Plano z = " + string(z_verif) + " m");
    xlabel("x [m]"); ylabel("y [m]");
    axis equal; grid minor;
    legend(leyendas, "Location","best");
    hold off;

end

%% Vistas laterales: y = 0.25, 0.15, 0.1

y_vals = [0.25, 0.15, 0.1];

for yi = 1:numel(y_vals)

    y_lat = y_vals(yi);
    puntos_alcanzables_lat    = [];
    puntos_no_alcanzables_lat = [];

    for c = 1:numel(cajas_x)

        x_range = cajas_x{c}(1):paso_verif:cajas_x{c}(2);
        z_range = z_lat_lim(1):paso_verif:z_lat_lim(2);

        for i = 1:numel(x_range)
            for k = 1:numel(z_range)

                it_count   = 0;
                alpha_rand = 0;
                beta_rand  = 0;
                gamma_rand = 0;

                while true
                    [ampl_sol, ~] = inv_kinematics(x_range(i), y_lat, z_range(k), ...
                        0*d+e+alpha_rand, 180*d+e+beta_rand, 0*d+e+gamma_rand, zeros(1,6), false, R, "NONE");

                    if size(ampl_sol,1) > 1
                        break;
                    else
                        it_count = it_count + 1;
                        alpha_rand = rand()*2*pi;
                        beta_rand  = rand()*2*pi;
                        gamma_rand = rand()*2*pi;

                        if it_count > 20
                            break;
                        end
                    end
                end

                if size(ampl_sol,1) > 1
                    puntos_alcanzables_lat    = [puntos_alcanzables_lat;    x_range(i) z_range(k)]; %#ok<AGROW>
                else
                    puntos_no_alcanzables_lat = [puntos_no_alcanzables_lat; x_range(i) z_range(k)]; %#ok<AGROW>
                end

            end
        end

    end

    figure;
    hold on;
    leyendas = strings(0);

    if ~isempty(puntos_alcanzables_lat)
        plot(puntos_alcanzables_lat(:,1), puntos_alcanzables_lat(:,2), ...
            "Marker",".","MarkerSize",10,"LineStyle","none","Color",[0 0.6 0]);
        leyendas(end+1) = "Puntos alcanzables";
    end
    if ~isempty(puntos_no_alcanzables_lat)
        plot(puntos_no_alcanzables_lat(:,1), puntos_no_alcanzables_lat(:,2), ...
            "Marker","x","MarkerSize",6,"LineStyle","none","Color","r");
        leyendas(end+1) = "Puntos no alcanzables";
    end

    rectangle("Position",[caja1_x(1), 0, diff(caja1_x), altura_caja], ...
        "EdgeColor","k","LineWidth",1.5);
    leyendas(end+1) = "Cajas de tarea";
    rectangle("Position",[caja2_x(1), 0, diff(caja2_x), altura_caja], ...
        "EdgeColor","k","LineWidth",1.5,"HandleVisibility","off");

    rectangle("Position",[-r_base, 0, 2*r_base, 0.2], ...
        "EdgeColor",[0.6 0.8 1], "FaceColor",[0.8 0.95 1], "LineWidth",1.5);
    leyendas(end+1) = "Base del robot";

    title("Vista lateral del espacio de trabajo - Plano y = " + string(y_lat) + " m");
    xlabel("x [m]"); ylabel("z [m]");
    axis equal; grid minor;
    legend(leyendas, "Location","best");
    hold off;

end

end
