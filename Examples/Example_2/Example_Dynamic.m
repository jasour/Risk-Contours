% This code computes risk bounded safe sets (Risk Contours) in the presence of uncertain safety constraints
% Ashkan Jasour, Weiqiao Han, Brian Williams,"Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments", Robotics: Science and Systems (RSS), 2021.
% Paper: http://www.roboticsproceedings.org/rss17/p069.pdf

%%
clc; clear all; close all
%% position: [x1 x2], time: t, uncertain parameters: w1, w2, w3
syms x1 x2 w1 w2 w3 t
%% uncertain trajectory (px,py) with uncertain parameters w2, w3
% w2: normal distribution on [mean,var] 
% w3: beta distribution [a,b] 
px=2-t+t^2+0.2*w2; 
py=-1+4*t-t^2+0.1*w3;
%% uncertain object g={(x1,x2): g(x1,x2,w1)>=0 } with uncertain parameter w1
% g : moving circle with uncertain radius w1 and uncertain center (px,py) at time t
% w1: uniform distribution on [l,u]
g=w1^2-(x1-px)^2-(x2-py)^2; 
% degree of uncertain object g 
dg=polynomialDegree(g);
%% statistics (moments) of probability distributions of uncertain parameters 
% see section II of paper

% w1 has Uniform distribution on [l,u]
u=0.4;l=0.3; 
% moments sequence of w1
m_w1=[1];for i=1:2*dg ;m_w1(i+1,1)=(1/(u-l))*((u^(i+1) - l^(i+1))/(i+1));end

% w2 has normal distribution on [mean,var] : 
mean=0; var=0.1; 
% moments sequence of w2
for k=0:2*dg; m_w2(k+1,1)=sqrt(var)^k*(-j*sqrt(2))^k*kummerU(-k/2, 1/2,-1/2*mean^2/var);end

% w3 has beta distribution [a,b] 
a=3;b=3; 
% moments sequence of w3
m_w3=[1];for k=1:2*dg; m_w3=[m_w3;(a+k-1)/(a+b+k-1)*m_w3(end) ]; end;

%% Computes the risk-bounded safe locations (Risk Contours) in the presence of uncertain moving object 
% Eq (9),(10), and Theorem I of the paper

% Calculate the first and Second order moments of uncertain object
Mg=[]; %List of first and second order moments
for dd=1:2
% Moment of order dd
Md=expand(g^dd);
% Replace moments of uncertain parameter w1
Md1=subs(Md,flip(w1.^[1:dd*dg].'),flip(m_w1(2:dd*dg+1))) ; 
% Replace moments of uncertain parameter w2
Md2=subs(Md1,flip(w2.^[1:dd*dg].'),flip(m_w2(2:dd*dg+1))) ; 
% Replace moments of uncertain parameter w3
Md3=subs(Md2,flip(w3.^[1:dd*dg].'),flip(m_w3(2:dd*dg+1))) ; 
Mg=[Mg;Md3];
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
Delta=0.1; % Acceptable risk level

clc;display('Done!'); display('Working on plots')
%% Plots: 
% Expected value of the uncertain trajectory (E[px1],E[px2]) (Dashed line).
% Risk-bounded safe locations at time steps t = 0, 0.5, 1.
% At each time t, for any point outside of the closed curve, risk
% of collision with the moving uncertain object is less or equal to Delta = 0.1.


figure; axis square; hold on
[x1,x2]=meshgrid([0.8:0.1:3],[-2:0.1:3]);
t=0;[C1,h1]=contour(x1,x2,eval(A_Cons_1),[Delta Delta],'r','ShowText','on','Linewidth',3); contour(x1,x2,eval(A_Cons_2),[0 0],'--b','Linewidth',1);
clabel(C1,h1,'FontSize',20)
t=0.5; [C2,h2]=contour(x1,x2,eval(A_Cons_1),[Delta Delta],'r','ShowText','on','Linewidth',3); contour(x1,x2,eval(A_Cons_2),[0 0],'--b','Linewidth',1);
clabel(C2,h2,'FontSize',20)
t=1; [C3,h3]=contour(x1,x2,eval(A_Cons_1),[Delta Delta],'r','ShowText','on','Linewidth',3); contour(x1,x2,eval(A_Cons_2),[0 0],'--b','Linewidth',1);
clabel(C3,h3,'FontSize',20)
t=[0:0.01:1];px=2-t+t.^2;py=-1+4*t-t.^2;plot(px,py,'k--','Linewidth',3)
t=0;px=2-t+t.^2;py=-1+4*t-t.^2;plot(px,py,'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
t=1;px=2-t+t.^2;py=-1+4*t-t.^2;plot(px,py,'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
title({'Expected value of the uncertain trajectory (dashed line).','Risk-bounded safe locations at time steps t = 0, 0.5, 1 (outside of the outer curve).'})


