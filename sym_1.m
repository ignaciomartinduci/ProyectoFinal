clc; clear; close all;
syms a b c d e f a1 a2 a3 a4 a5 a6 d1 d2 d3 d4 d5 d6 r11 r12 r13 r21 r22 r23 r31 r32 r33 x y z

%% RHS

T12 = [ sin(b)  cos(b)  0   a2*sin(b)
        -cos(b) sin(b)  0   -a2*cos(b)
        0       0       1   0
        0       0       0   1];

T23 = [ cos(c)  -sin(c)     0   a3*cos(c);
        sin(c)   cos(c)     0   a3*sin(c);
        0       0           1   d3;
        0       0           0   1 ];

T34 = [ -sin(d)  0   cos(d)   0
        cos(d) 0   sin(d)   0
        0       1   0         0
        0       0   0         1];

T45 = [ cos(e)  0   -sin(e)     0
        sin(e)  0   cos(e)      0
        0       -1  0           d5
        0       0   0           1];

RHS = simplify(T12 * T23 * T34 * T45)

%% LHS

T06 = [r11 r12 r13 x
       r21 r22 r23 y
       r31 r32 r33 z
       0   0   0   1];

T01_inv = [cos(a)       sin(a)      0   0
           0            0           -1  0
           -sin(a)      cos(a)      0   0
            0           0           0   1];

T56_inv = [ cos(f)  sin(f)  0 0
            -sin(f) cos(f)  0 0
            0       0       1 0
            0       0       0 1];

LHS = simplify(T01_inv*T06*T56_inv)


