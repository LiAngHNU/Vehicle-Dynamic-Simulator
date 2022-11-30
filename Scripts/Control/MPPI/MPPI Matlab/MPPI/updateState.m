%% 更新状态
function [] = updateState(state, v,Delta_delta,dt,refDelta)
persistent a b c d e f fl fr rl rr

x = state(1);
y = state(2);
yaw = state(3);

lf = 1.65;lf2 = lf +0.5;
lr = 1.35;lr2 = lr + 0.5;
w = 0.8;
Delta = refDelta + Delta_delta;

    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;


seta_f = atan(w/lf);seta_r = atan(w/lr);
seta_f2 = atan(w/lf2);seta_r2 = atan(w/lr2);
l_f = sqrt(lf^2+w^2);l_r = sqrt(lr^2+w^2);
l_f2 = sqrt(lf2^2+w^2);l_r2 = sqrt(lr2^2+w^2);
x_qianzhou = x + lf*cos(yaw); y_qianzhou = y + lf*sin(yaw);
%车前轴线
x_fr = x+l_f*cos(yaw-seta_f); y_fr = y+l_f*sin(yaw-seta_f);
x_fl = x+l_f*cos(yaw+seta_f); y_fl = y+l_f*sin(yaw+seta_f);
%车头线
x_fr2 = x+l_f2*cos(yaw-seta_f2); y_fr2 = y+l_f2*sin(yaw-seta_f2);
x_fl2 = x+l_f2*cos(yaw+seta_f2); y_fl2 = y+l_f2*sin(yaw+seta_f2);

%车后轴线
x_rr = x-l_r*cos(yaw+seta_r); y_rr = y-l_r*sin(yaw+seta_r);
x_rl = x-l_r*cos(yaw-seta_r); y_rl = y-l_r*sin(yaw-seta_r);
%车尾线
x_rr2 = x-l_r2*cos(yaw+seta_r2); y_rr2 = y-l_r2*sin(yaw+seta_r2);
x_rl2 = x-l_r2*cos(yaw-seta_r2); y_rl2 = y-l_r2*sin(yaw-seta_r2);
%%%
%车轮绘制
%后轮->
tire_l = 0.3;
%右后
x_t_rr1 = x_rr + tire_l*cos(yaw);y_t_rr1 = y_rr + tire_l*sin(yaw);
x_t_rr2 = x_rr - tire_l*cos(yaw);y_t_rr2 = y_rr - tire_l*sin(yaw);
%左后
x_t_rl1 = x_rl + tire_l*cos(yaw);y_t_rl1 = y_rl + tire_l*sin(yaw);
x_t_rl2 = x_rl - tire_l*cos(yaw);y_t_rl2 = y_rl - tire_l*sin(yaw);
%前轮->
%右前
x_t_fr1 = x_fr + tire_l*cos(yaw+Delta);y_t_fr1 = y_fr + tire_l*sin(yaw+Delta);
x_t_fr2 = x_fr - tire_l*cos(yaw+Delta);y_t_fr2 = y_fr - tire_l*sin(yaw+Delta);
%左前
x_t_fl1 = x_fl + tire_l*cos(yaw+Delta);y_t_fl1 = y_fl + tire_l*sin(yaw+Delta);
x_t_fl2 = x_fl - tire_l*cos(yaw+Delta);y_t_fl2 = y_fl - tire_l*sin(yaw+Delta);
%%%
figure(1);
delete(a);delete(b);delete(c);delete(d);delete(e);delete(f);delete(fl);delete(fr);delete(rl);delete(rr);
%标记车前轴中心

    scatter(x_qianzhou,y_qianzhou,500,'b.');


a = line([x_fl x_fr],[y_fl y_fr],'LineWidth',2,'color','r');
b = line([x_rr2 x_fr2],[y_rr2 y_fr2],'LineWidth',2,'color','r');
c = line([x_rr x_rl],[y_rr y_rl],'LineWidth',2,'color','r');
d = line([x_fl2 x_rl2],[y_fl2 y_rl2],'LineWidth',2,'color','r');
e = line([x_fl2 x_fr2],[y_fl2 y_fr2],'LineWidth',2,'color','r');
f = line([x_rr2 x_rl2],[y_rr2 y_rl2],'LineWidth',2,'color','r');
%车轮绘制
fl = line([x_t_fl2 x_t_fl1],[y_t_fl2 y_t_fl1],'LineWidth',5,'color','k');
fr = line([x_t_fr2 x_t_fr1],[y_t_fr2 y_t_fr1],'LineWidth',5,'color','k');
rl = line([x_t_rl2 x_t_rl1],[y_t_rl2 y_t_rl1],'LineWidth',5,'color','k');
rr = line([x_t_rr2 x_t_rr1],[y_t_rr2 y_t_rr1],'LineWidth',5,'color','k');
%%%
hold on
%pause(0.01);


end