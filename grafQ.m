function [] = grafQ(q_traj, qd_traj, qdd_traj)

    figure;
    hold on
        subplot(6,1,1)
        plot(q_traj(:,1))
        subplot(6,1,2)
        plot(q_traj(:,2))
        subplot(6,1,3)
        plot(q_traj(:,3))
        subplot(6,1,4)
        plot(q_traj(:,4))
        subplot(6,1,5)
        plot(q_traj(:,5))
        subplot(6,1,6)
        plot(q_traj(:,6))
        title("q_{traj}")
        grid minor;
    hold off

    figure;
    hold on
        subplot(6,1,1)
        plot(qd_traj(:,1))
        subplot(6,1,2)
        plot(qd_traj(:,2))
        subplot(6,1,3)
        plot(qd_traj(:,3))
        subplot(6,1,4)
        plot(qd_traj(:,4))
        subplot(6,1,5)
        plot(qd_traj(:,5))
        subplot(6,1,6)
        plot(qd_traj(:,6))
        title("qd_{traj}")
        grid minor;
    hold off

    figure;
    hold on
        subplot(6,1,1)
        plot(qdd_traj(:,1))
        subplot(6,1,2)
        plot(qdd_traj(:,2))
        subplot(6,1,3)
        plot(qdd_traj(:,3))
        subplot(6,1,4)
        plot(qdd_traj(:,4))
        subplot(6,1,5)
        plot(qdd_traj(:,5))
        subplot(6,1,6)
        plot(qdd_traj(:,6))
        title("qdd_{traj}")
        grid minor;
    hold off

end