function [] = solutions(R, all_sol_q)

%% 1 - Animación de soluciones geométricas

all_sol = zeros(size(all_sol_q));

for i=1:length(all_sol(:,1))

    T = R.fkine(all_sol(i,:));
    all_sol(i,1:3) = T.t;
    all_sol(i,4:6) = tr2rpy(T,'zyx');

end

d = pi/180;
dt = 0.01;

q0 = all_sol(1,:);
cpoints_modes = zeros(length(all_sol(:,1)), 7);

cpoints_modes(:,1:6)=all_sol;

qdmax = [180 180 180 360 360 360]*d;
qddmax = [800 800 800 800 800 800]*d;
fps = 24;
dt = 1/fps;

q_traj = gen_traj(R,cpoints_modes, q0, qdmax, qddmax, dt, 0);


end