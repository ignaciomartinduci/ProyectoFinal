function [] = grafQaE(R, q_traj)

    N = length(q_traj(:,1));

    T = zeros(4,4,N);

    x = zeros(N);
    y = zeros(N);
    z = zeros(N);
    roll = zeros(N);
    pitch = zeros(N);
    yaw = zeros(N);
    

    for i=1:length(q_traj(:,1))

        T(:,:,i) = R.fkine(q_traj(i,:));
        x(i) = T(1,4,i);
        y(i) = T(2,4,i);
        z(i) = T(3,4,i);

        rpy = tr2rpy(T(:,:,i),'zyx');

        roll(i) = rpy(1);
        pitch(i) = rpy(2);
        yaw(i) = rpy(3);

    end

    figure;
        hold on;
        subplot(6,1,1);
        plot(x); grid minor; xlabel('t'); ylabel('x');
        subplot(6,1,2);
        plot(y); grid minor; xlabel('t'); ylabel('y');
        subplot(6,1,3);
        plot(z); grid minor; xlabel('t'); ylabel('z');
        subplot(6,1,4);
        plot(roll); grid minor; xlabel('t'); ylabel('roll');
        subplot(6,1,5);
        plot(pitch); grid minor; xlabel('t'); ylabel('pitch');
        subplot(6,1,6);
        plot(yaw); grid minor; xlabel('t'); ylabel('yaw');
        sgtitle('Trayectoria cartesiana');
        hold off;

end