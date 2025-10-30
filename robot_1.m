    d_b = 0.150+0.100; % ajustar
    d_1 = 0;
    d_2 = 0;
    d_3 = 0.150;
    d_4 = 0;
    d_5 = 0.100;
    d_6 = 0;
    TCP = 0.100+0.145; % ajustar%
    
    a_1 = 0;
    a_2 = 0.250;
    a_3 = 0.225;
    a_4 = 0;
    a_5 = 0;
    a_6 = 0;
    
    % DH 1
    dh = [
        0       d_1     a_1       -pi/2;
        0       d_2     a_2     0;
        0       d_3     a_3     0;
        0       d_4     a_4       pi/2;
        0       d_5     a_5       -pi/2;
        0       d_6     a_6       0

    ];


    
    q_offset = [0, -pi/2, 0, pi/2, 0, 0];
    qdmax = [pi pi pi 2*pi 2*pi 2*pi]; % Velocidades máximas de cada articulación
    
    R = SerialLink(dh,'name','DR - 1');
    R.offset = q_offset;
    base = transl(0,0,d_b);
    tool = transl(0,0,TCP);
    R.base = base;
    R.tool = tool;
    R_qdmax = qdmax;
    
    R.qlim(1,1:2) = [-360,  360]*pi/180; 
    R.qlim(2,1:2) = [-360,  360]*pi/180;
    R.qlim(3,1:2) = [-360,  360]*pi/180;
    R.qlim(4,1:2) = [-360,  360]*pi/180;
    R.qlim(5,1:2) = [-360,  360]*pi/180;
    R.qlim(6,1:2) = [-1e6,1e6]*pi/180;
    
    q_0 = [0,0,0,0,0,0]*pi/180;
    
    %figure;
    % R.teach(q_0,"scale",0.5)
    % input("-> Detención por plot - enter para continuar");
    % R.plot([ -4.7878   -0.0206    5.7773   -2.0058    1.0715   -6.2832], "scale", 0.5)
    % input("")
    

