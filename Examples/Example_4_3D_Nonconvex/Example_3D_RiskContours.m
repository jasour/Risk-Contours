% This code computes risk bounded safe sets (Risk Contours) in the presence of uncertain safety constraints
% Ashkan Jasour, Weiqiao Han, Brian Williams,"Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments", Robotics: Science and Systems (RSS), 2021.
% Paper: http://www.roboticsproceedings.org/rss17/p069.pdf

%%
clc; clear all; close all
%% position: [x1 x2], time: t, uncertain parameters: w1, w2, w3
syms x1 x2 x3 w

%% uncertain object g={(x1,x2): g(x1,x2,w)>=0 } with uncertain parameter w
g=0.94-0.002.*x1-0.004.*x2-0.04.*x3-0.38.*x1.^2+0.04.*x1.*x2-0.31.*x2.^2-0.05.*x1.*x3-0.01.*x2.*x3-0.4.*x3.^2-0.1.*x1.^3-0.02.*x1.^2.*x2+0.09.*x1.*x2.^2-0.05.*x2.^3+0.14.*x1.^2.*x3-1.83.*x1.*x2.*x3+0.11.*x2.^2.*x3-0.1.*x1.*x3.^2+0.12.*x2.*x3.^2+0.34.*x3.^3-0.32.*x1.^4-0.13.*x1.^3.*x2+0.48.*x1.^2.*x2.^2+0.11.*x1.*x2.^3-0.34.*x2.^4+0.03.*x1.^3.*x3+0.01.*x1.^2.*x2.*x3-0.005.*x1.*x2.^2.*x3-0.05.*x2.^3.*x3+0.54.*x1.^2.*x3.^2-0.06.*x1.*x2.*x3.^2+0.48.*x2.^2.*x3.^2+0.008.*x1.*x3.^3+0.06.*x2.*x3.^3-0.3.*x3.^4+0.12.*x1.^5+0.005.*x1.^4.*x2-0.1.*x1.^3.*x2.^2+0.007.*x1.^2.*x2.^3+0.005.*x1.*x2.^4+0.071.*x2.^5-0.02.*x1.^4.*x3+0.73.*x1.^3.*x2.*x3-0.07.*x1.^2.*x2.^2.*x3+0.72.*x1.*x2.^3.*x3-0.20.*x2.^4.*x3+0.03.*x1.^3.*x3.^2-0.01.*x1.^2.*x2.*x3.^2+0.02.*x1.*x2.^2.*x3.^2-0.05.*x2.^3.*x3.^2-0.07.*x1.^2.*x3.^3+0.73.*x1.*x2.*x3.^3+0.09.*x2.^2.*x3.^3+0.03.*x1.*x3.^4-0.06.*x2.*x3.^4-0.31.*x3.^5-w-0.84;
dg=polynomialDegree(g); % max degree of uncertain object g
%% statistics (moments) of probability distributions of uncertain parameters 
% see section II of paper

% w: uncertain parameter w~Normal(mean,variance)
mean=0.1; var=0.001; for k=0:2*dg; m_w(k+1,1)=sqrt(var)^k*(-j*sqrt(2))^k*kummerU(-k/2, 1/2,-1/2*mean^2/var);end

%% Computes the risk-bounded safe locations (Risk Contours) in the presence of uncertain moving object 
% Eq (9),(10), and Theorem I of the paper

% Calculate the first and Second order moments of uncertain object
Mg=[]; %List of first and second order moments
for dd=1:2
% Moment of order dd
Md=expand(g^dd);
% Replace moments of uncertain parameter w
Md1=subs(Md,flip(w.^[1:dd*dg].'),flip(m_w(2:dd*dg+1))) ; 
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
clc;display('Done!');display('Working on plots!')

%% Plot

[x1,x2,x3]=meshgrid([-1:0.03:1],[-1:0.03:1],[-1:0.03:1]);
P=eval(A_Cons_1);

figure; hold on

Delta=0.5; h = patch(isosurface(x1,x2,x3,P,Delta));
isonormals(x1,x2,x3,P,h);set(h,'FaceColor','green','EdgeColor','none','FaceAlpha',1); % red
lighting phong; view(3); axis tight vis3d; camlight ;axis normal

Delta=0.3;h = patch(isosurface(x1,x2,x3,P,Delta));
isonormals(x1,x2,x3,P,h);set(h,'FaceColor','black','EdgeColor','none','FaceAlpha',0.4); % red
lighting phong; view(3); axis tight vis3d; camlight ;axis normal

Delta=0.1;h = patch(isosurface(x1,x2,x3,P,Delta));
isonormals(x1,x2,x3,P,h);set(h,'FaceColor','blue','EdgeColor','none','FaceAlpha',0.3); % red
lighting phong; view(3); axis tight vis3d; camlight ;axis normal


Delta=0.05;h = patch(isosurface(x1,x2,x3,P,Delta));
isonormals(x1,x2,x3,P,h);set(h,'FaceColor','red','EdgeColor','none','FaceAlpha',0.1); % red
lighting phong; view(3); axis tight vis3d; camlight ;axis normal

legend('Delta=0.5','Delta=0.3','Delta=0.1','Delta=0.05')
grid on; axis square; set(gca,'fontsize',25);xlim([-1 1]);ylim([-1 1]);zlim([-1 1])
xlabel('$x_1$','Interpreter','latex', 'FontSize',25); ylabel('$x_2$','Interpreter','latex', 'FontSize',25);view(-72,10)

