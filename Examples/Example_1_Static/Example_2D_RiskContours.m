% This code computes risk bounded safe sets (Risk Contours) in the presence of uncertain safety constraints
% Ashkan Jasour, Weiqiao Han, Brian Williams,"Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments", Robotics: Science and Systems (RSS), 2021.
% Paper: http://www.roboticsproceedings.org/rss17/p069.pdf

%%
clc; clear all; close all
%% position: [x1 x2], uncertain parameter: w1
syms x1 x2 w1

%% uncertain object g={(x1,x2): g(x1,x2,w1)>=0 } with uncertain parameter w1
% g : circle with uncertain radius w1
% w1: uniform distribution on [l,u]
g=w1^2-(x1 - 0)^2-(x2 - 0 )^2; 
% degree of uncertain object g 
dg=polynomialDegree(g);
%% statistics (moments) of probability distributions of uncertain parameters 
% see section II of paper

% w1 has Uniform distribution on [l,u]
u=0.4;l=0.3; 
% moments sequence of w1
m_w1=[1];for i=1:2*dg ;m_w1(i+1,1)=(1/(u-l))*((u^(i+1) - l^(i+1))/(i+1));end

%% Computes the risk-bounded safe locations (Risk Contours) in the presence of uncertain moving object 
% Eq (9),(10), and Theorem I of the paper

% Calculate the first and Second order moments of uncertain object
Mg=[]; %List of first and second order moments
for dd=1:2
% Moment of order dd
Md=expand(g^dd);
% Replace moments of uncertain parameter w1
Md1=subs(Md,flip(w1.^[1:dd*dg].'),flip(m_w1(2:dd*dg+1))) ; 
Mg=[Mg;Md1];
end

%% A) Risk-bounded safe locations (Risk Contours) in the presence of uncertain moving object whose risk <= Delta 
% safe set= {(x1,x2):  Cons_1 <= Delta , Cons_2 <=0} ( Eq(12) in the paper)
% *********************
% Paper: Ashkan Jasour, Weiqiao Han, Brian Williams,"Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments", Robotics: Science and Systems (RSS), 2021.
% http://www.roboticsproceedings.org/rss17/p069.pdf
A_Cons_1=(Mg(2)-Mg(1)^2)/Mg(2);   % <= Delta
A_Cons_2=Mg(1); % This constraint should be <=0  since being safe is <=0  (according to safety constraints)

%% B) Risk-bounded safe locations (Risk Contours) in the presence of uncertain moving object whose risk <= Delta
% safe set= {(x1,x2):  Cons_1 <= Delta , Cons_2 >=0, Cons_3 >= 0 } ( Eq(6) in the paper)
% *********************
% Paper: Weiqiao Han, Ashkan Jasour, Brian Williams,"Non-Gaussian Risk Bounded Trajectory Optimization for Stochastic Nonlinear Systems in Uncertain Environments", 39th IEEE Conference on Robotics and Automation (ICRA), 2022.
% https://arxiv.org/pdf/2203.03038.pdf
B_Cons_1=(4/9)*(Mg(2)-Mg(1)^2)/Mg(2);  % <= Delta
B_Cons_2 = Mg(1)^2-5/8*Mg(2);  % >= 0 
B_Cons_3=Mg(1); % This constraint should be <=0  since being safe is <=0  according to the defined safety constraints. In the papre, being safe is >=0; Hence, Cons_3 >=0.


%%
Delta=0.01; % Acceptable risk level

clc;display('Done!'); display('Working on plots')

%% Plots: 
% Risk contour: outside of the outer curve, risk of collision with the uncertain object is less or equal to Delta = 0.1.
[x1,x2]=meshgrid([-1:0.01:1],[-1:0.01:1]);

% Plot A
figure(1); hold on;
surf(x1,x2,eval(A_Cons_1),'FaceColor','red','EdgeColor','none','FaceAlpha',0.8);
surf(x1,x2,eval(A_Cons_2),'FaceColor','blue','EdgeColor','none','FaceAlpha',0.8);
title({'Approach A: level sets of the risk contours constraints','red: Cons 1 <= Delta, blue: Cons 2 <= 0'})
view(10,15); camlight(0,0); lighting gouraud; zlim([-1 1])

figure(2); axis square; hold on
[C_A,h]=contour(x1,x2,eval(A_Cons_1),[Delta Delta],'r','ShowText','on','Linewidth',3); contour(x1,x2,eval(A_Cons_2),[0 0],'--b','Linewidth',1);
clabel(C_A,h,'FontSize',20)
title({'Approach A: Risk-bounded safe locations(outside of the outer curve)'})
figure(1); plot3(C_A(1,2:end),C_A(2,2:end),Delta*ones(1,size(C_A(2,2:end),2)),'y','LineWidth',2)
figure(2)


% Plot B
figure(3); hold on;
surf(x1,x2,eval(B_Cons_1),'FaceColor','red','EdgeColor','none','FaceAlpha',0.8);
surfc(x1,x2,eval(B_Cons_2),'FaceColor','magenta','EdgeColor','none','FaceAlpha',0.8);
surf(x1,x2,eval(B_Cons_3),'FaceColor','blue','EdgeColor','none','FaceAlpha',0.8);
title({'Approach B: level sets of the risk contours constraints','red: Cons 1 <= Delta, magenta: Cons 2 >= 0, blue: Cons 3 <= 0'})
view(10,15); camlight(0,0); lighting gouraud; zlim([-1 1])

figure(4); axis square; hold on
[C_B,h]=contour(x1,x2,eval(B_Cons_1),[Delta Delta],'r','ShowText','on','Linewidth',3); contour(x1,x2,eval(B_Cons_2),[0 0],'.m','Linewidth',1); contour(x1,x2,eval(B_Cons_3),[0 0],'--b','Linewidth',1);
clabel(C_B,h,'FontSize',20)
title({'Approach B: Risk-bounded safe locations(outside of the outer curve)'})
figure(3); plot3(C_B(1,2:end),C_B(2,2:end),Delta*ones(1,size(C_B(2,2:end),2)),'y','LineWidth',2)
figure(4)

% Compare A and B
figure(5); hold on
plot3(C_A(1,2:end),C_A(2,2:end),Delta*ones(1,size(C_A(2,2:end),2)),'r','LineWidth',2)
plot3(C_B(1,2:end),C_B(2,2:end),Delta*ones(1,size(C_B(2,2:end),2)),'b','LineWidth',2)
legend('Approach A','Approach B');
title({'Risk-bounded safe locations'})

