d_b = 0.250; % ajustar
d_1 = d_b;
d_2 = 0;
d_3 = 0.150+0.105+0.040;
d_4 = 0;
d_5 = 0.100;
d_6 = 0;
TCP = 0.145+0.045;% Longitud gripper + distancia origen sistema 5 a punto de colocación de la tool

a_1 = 0;
a_2 = 0.250;
a_3 = 0.250;
a_4 = 0;
a_5 = 0;
a_6 = 0;

% DH 1
dh = [
    0       d_1     a_1         -pi/2;
    0       d_2     a_2         0;
    0       d_3     a_3         0;
    0       d_4     a_4         pi/2;
    0       d_5     a_5         -pi/2;
    0       d_6     a_6         0

    ];



q_offset = [0, -pi/2, 0, pi/2, 0, 0];
qdmax = [pi pi pi pi pi pi]; % Velocidades máximas de cada articulación
qddmax = [2*pi 2*pi 2*pi 2*pi 2*pi 2*pi];

R = SerialLink(dh,'name','DR - 1');
R.offset = q_offset;
base = transl(0,0,0);
tool = transl(0,0,TCP);
R.base = base;
R.tool = tool;
R_qdmax = qdmax;
R_qddmax = qddmax;

R.qlim(1,1:2) = [-360,  360]*pi/180;
R.qlim(2,1:2) = [-360,  360]*pi/180;
R.qlim(3,1:2) = [-360,  360]*pi/180;
R.qlim(4,1:2) = [-360,  360]*pi/180;
R.qlim(5,1:2) = [-360,  360]*pi/180;
R.qlim(6,1:2) = [-1e6,1e6]*pi/180;

q_0 = [15,20,-15,20,45,5]*pi/180;
q_0 = [-pi/5 -pi/5 -pi/5 -pi/5 -pi/5 -pi/5];
q_0 = [0 0 0 0 0 0];

% path = [pwd, '/models/CAD2'];

% R.plot3d(q_0, ...
%     'path', path,'alpha',0.25);
% axis equal
% axis manual
% hold on
% trplot(R.A(1:5, q_0).T, 'frame', '5', 'length', 0.1)
% hold on   
% trplot(R.A(1:4, q_0).T, 'frame', '4', 'length', 0.08)
% hold on
% trplot(R.A(1:3, q_0).T, 'frame', '3', 'length', 0.05)
% hold on
% trplot(R.A(1:2, q_0).T, 'frame', '2', 'length', 0.05)
% hold on
% trplot(R.A(1:1, q_0).T, 'frame', '1', 'length', 0.05)
% hold on
% trplot(R.A(1:0, q_0).T, 'frame', '0', 'length', 0.05)
% camproj orthographic
% campos([2 2 1.5])     % posición de la cámara
% camtarget([0 0 0.5])  % hacia dónde mira
% camup([0 0 1])        % eje vertical

% camva(5)             % zoom

% camlight headlight
% lighting gouraud




