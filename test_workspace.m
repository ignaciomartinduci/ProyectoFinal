function [] = test_workspace(R)

    step = 5*pi/180;


    q1 = R.qlim(1,1):step:R.qlim(1,2);
    q2 = pi/2;
    q3 = 0;
    q4 = -pi/2;
    q5 = -pi/2;
    q6 = 0;

    puntos_alcanzados = zeros(numel(q1),2);
    idx = 0;


    for i = 1:numel(q1)

        idx = idx + 1;

        q = [q1(i) q2 q3 q4 q5 q6];

        T = R.fkine(q);
        p = T.t;

        x = p(1);
        y = p(2);

        puntos_alcanzados(idx,:) = [x y];

    end

figure;

% Plot puntos
plot(puntos_alcanzados(:,1), puntos_alcanzados(:,2), ...
    "Marker","*","LineStyle","none");
hold on;

% Centro del workspace
xc = mean(puntos_alcanzados(:,1));
yc = mean(puntos_alcanzados(:,2));

% Parámetro angular
theta = linspace(0, 2*pi, 200);

% Radios
r_max = 0.5;        % 500 mm
r_tarea = 0.34515;  % 345.15 mm

% Círculo rango máximo
x_max = xc + r_max*cos(theta);
y_max = yc + r_max*sin(theta);
plot(x_max, y_max, 'r-', 'LineWidth', 1.5);

% Círculo alcance tarea
x_t = xc + r_tarea*cos(theta);
y_t = yc + r_tarea*sin(theta);
plot(x_t, y_t, 'g--', 'LineWidth', 1.5);

% Labels
title("Comprobación gruesa del espacio de trabajo SB-S4");
xlabel("x [m]");
ylabel("y [m]");

% Mantener proporción
axis equal;

grid minor;

legend("Puntos alcanzados", ...
       "Rango máximo (500 mm)", ...
       "Alcance tarea (345.15 mm)", ...
       "Location","best");

hold off;

end