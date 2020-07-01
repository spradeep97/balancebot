clc
clear all
close all

cw1 = load('state_log_cw1.csv');
cw2 = load('state_log_cw2.csv');
cw3 = load('state_log_cw3.csv');
cw4 = load('state_log_cw4.csv');
cw5 = load('state_log_cw5.csv');

ccw1 = load('state_log_ccw1.csv');
ccw2 = load('state_log_ccw2.csv');
ccw3 = load('state_log_ccw3.csv');
ccw4 = load('state_log_ccw4.csv');
ccw5 = load('state_log_ccw5.csv');

x_cw1 = cw1(end,3);
y_cw1 = cw1(end,4);
x_cw2 = cw2(end,3);
y_cw2 = cw2(end,4);
x_cw3 = cw3(end,3);
y_cw3 = cw3(end,4);
x_cw4 = cw4(end,3);
y_cw4 = cw4(end,4);
x_cw5 = cw5(end,3);
y_cw5 = cw5(end,4);

x_ccw1 = ccw1(end,4);
y_ccw1 = ccw1(end,3);
x_ccw2 = ccw2(end,4);
y_ccw2 = ccw2(end,3);
x_ccw3 = ccw3(end,4);
y_ccw3 = ccw3(end,3);
x_ccw4 = ccw4(end,4);
y_ccw4 = ccw4(end,3);
x_ccw5 = ccw5(end,4);
y_ccw5 = ccw5(end,3);

x_cw_sum = x_cw1 + x_cw2 + x_cw3 + x_cw4 + x_cw5;
x_cg_cw = x_cw_sum/5;
y_cw_sum = y_cw1 + y_cw2 + y_cw3 + y_cw4 + y_cw5;
y_cg_cw = y_cw_sum/5;

x_ccw_sum = x_ccw1 + x_ccw2 + x_ccw3 + x_ccw4 + x_ccw5;
x_cg_ccw = x_ccw_sum/5;
y_ccw_sum = y_ccw1 + y_ccw2 + y_ccw3 + y_ccw4 + y_ccw5;
y_cg_ccw = y_ccw_sum/5;

L = 2.4384;
alpha1 = (x_cg_cw + x_cg_ccw) * 180 / (-pi * 4 * L)
alpha2 = (y_cg_cw - y_cg_ccw) * 180 / (-pi * 4 * L)
alpha = (alpha1 + alpha2) / 2;
Eb = 90 / (90 - alpha2);

beta1 = (x_cg_cw - x_cg_ccw) * 180 / (-pi * 4 * L)
beta2 = (y_cg_cw + y_cg_ccw) * 180 / (-pi * 4 * L)
beta = (beta1 + beta2) / 2;
R = L / (2 * sin(beta2 / 2));
b = 0.212;
Ed = (R + b / 2) / (R - b / 2)
