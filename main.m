clc; clear; close all;

addpath("./common");
addpath("./rtb");
addpath("smtb");

%% ======================

flags;
robot_1;

%% ======================
    
if TEST_WORKSPACE

    test_workspace(R);

end

if TEST_INV_KINEMATICS
    
    test_inv_kinematics(R, TEST_LOOP_IK);

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

