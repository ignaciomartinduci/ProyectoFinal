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

if CAD_DESIGN == 2
    cad_path = fullfile(pwd, 'models', 'CAD2');
else
    cad_path = fullfile(pwd, 'models', 'CAD');
end

%% ======================

disp("== INICIANDO PROGRAMA ==");

if TEST_WORKSPACE

    test_workspace(R);

end

if TEST_INV_KINEMATICS

    test_inv_kinematics(R, TEST_LOOP_IK, PRINT_SOLUTIONS);

end

if SINGULARITIES

    test_singularities(R);

end

if TEST_GEN_TRAJ

    test_gen_traj(R, cad_path);
end

if TAREA
    tarea(R, cad_path);
end

if IK_COMPARISON
    cad_path_nb = fullfile(pwd, 'models', 'CAD_NO_BOXES');
    ik_comparison(R, cad_path_nb);
end

