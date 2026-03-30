clc; clear; close all;

addpath("./third_party/common");
addpath("./third_party/rtb");
addpath("./third_party/smtb");
addpath("./models");
addpath("./modules");
addpath("./tests");
addpath("./visualization");

%% ======================

flags;
robot_1;

%% ======================

disp("== INICIANDO PROGRAMA ==");   
    
if TEST_WORKSPACE

    test_workspace(R);

end

if TEST_INV_KINEMATICS
    
    test_inv_kinematics(R, TEST_LOOP_IK, PRINT_SOLUTIONS);

end

if SINGULARITIES

    singularities(R, PLOT_SINGULARITIES);
    
end

if TEST_GEN_TRAJ 

    test_gen_traj(R);
end

if TAREA
    tarea(R);
end

