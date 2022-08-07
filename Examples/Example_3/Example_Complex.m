% This code computes risk bounded safe sets (Risk Contours) in the presence of uncertain safety constraints
% Ashkan Jasour, Weiqiao Han, Brian Williams,"Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments", Robotics: Science and Systems (RSS), 2021.
% Paper: http://www.roboticsproceedings.org/rss17/p069.pdf

%%
clc; clear all; close all
%% position: [x1 x2], time: t, uncertain parameters: w1, w2, w3
syms x1 x2 w

%% uncertain object g={(x1,x2): g(x1,x2,w)>=0 } with uncertain parameter w
C=[0.83,-0.21,-0.08,0.06,0.6,-0.41,0.6,1.87,-0.85,-0.07,-0.47,-0.57,0.17,-0.14,-0.06,-0.42,-1.18,0.30,-0.65,0.69,0.01];
pow=[];for i=0:5; pow=[pow;genpow(2,i)];end
g= sum(1*C'.*(1*x1).^pow(:,1).*(1*x2).^pow(:,2))-(0.76 + 0.1*w); 
dg=polynomialDegree(g); % max degree of uncertain object g
%% statistics (moments) of probability distributions of uncertain parameters 
% see section II of paper

% w: uncertain parameter has Beta distribution with parameters (a,b)
a=9;b=0.5; 
% moments sequence of uncertain parameter w
m_w=[1];for k=1:2*dg; m_w=[m_w;(a+k-1)/(a+b+k-1)*m_w(end) ]; end;

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


% Compare A and B
figure(5); hold on; axis square; Fs1=20;l=-1;u=1;
Delta=[0.3 0.1 0.05 0.02 0.01]; % Risk levles
sgtitle({'Risk-bounded safe locations for different risk levels $\Delta$ (outside of the outer curve)','red: Approach A, blue: Approach B'},'Interpreter','latex')

[x1,x2]=meshgrid([-1:0.01:1],[-1:0.01:1]);
subplot(2,5,1);grid on; hold on; axis square
        contour(x1,x2,eval(A_Cons_1),[Delta(1) Delta(1)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        subtitle('$\Delta=0.3$','Interpreter','latex');
subplot(2,5,2);grid on; hold on; axis square
        contour(x1,x2,eval(A_Cons_1),[Delta(2) Delta(2)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        subtitle('$\Delta=0.1$','Interpreter','latex');
subplot(2,5,3);grid on; hold on; axis square
        contour(x1,x2,eval(A_Cons_1),[Delta(3) Delta(3)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        subtitle('$\Delta=0.05$','Interpreter','latex');
subplot(2,5,4);grid on; hold on; axis square
        contour(x1,x2,eval(A_Cons_1),[Delta(4) Delta(4)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        subtitle('$\Delta=0.02$','Interpreter','latex');
subplot(2,5,5);grid on; hold on; axis square
        contour(x1,x2,eval(A_Cons_1),[Delta(5) Delta(5)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        subtitle('$\Delta=0.01$','Interpreter','latex');
        


subplot(2,5,6);grid on; hold on; axis square
        contour(x1,x2,eval(B_Cons_1),[Delta(1) Delta(1)],'b','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        subtitle('$\Delta=0.3$','Interpreter','latex');
subplot(2,5,7);grid on; hold on; axis square
        contour(x1,x2,eval(B_Cons_1),[Delta(2) Delta(2)],'b','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        subtitle('$\Delta=0.1$','Interpreter','latex');
subplot(2,5,8);grid on; hold on; axis square
        contour(x1,x2,eval(A_Cons_1),[Delta(3) Delta(3)],'b','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        subtitle('$\Delta=0.05$','Interpreter','latex');
subplot(2,5,9);grid on; hold on; axis square
        contour(x1,x2,eval(B_Cons_1),[Delta(4) Delta(4)],'b','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        subtitle('$\Delta=0.02$','Interpreter','latex');
subplot(2,5,10);grid on; hold on; axis square
        contour(x1,x2,eval(B_Cons_1),[Delta(5) Delta(5)],'b','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        subtitle('$\Delta=0.01$','Interpreter','latex');
        
% plot uncertain object for different values of uncertainty w
Example_Complex_Plot