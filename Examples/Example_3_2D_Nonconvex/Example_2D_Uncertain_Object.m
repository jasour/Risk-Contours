clc;%clear;close all
%% Uncertain Obstacle : Eq(4)
% obstacle g(x1,x2,w)>=0 where w is probabilistic uncertainty
%% position: [x1 x2], uncertain parameter: w
syms x1 x2 w
%% uncertain object g={(x1,x2): g(x1,x2,w)>=0 } with uncertain parameter w
pow=[];for i=0:5; pow=[pow;genpow(2,i)];end
C=[0.83,-0.21,-0.08,0.06,0.6,-0.41,0.6,1.87,-0.85,-0.07,-0.47,-0.57,0.17,-0.14,-0.06,-0.42,-1.18,0.30,-0.65,0.69,0.01];
g= sum(1*C'.*(1*x1).^pow(:,1).*(1*x2).^pow(:,2))-(0.76 + 0.1*w);

%% Plot: geometry of the uncertain object for different values of uncertainty w
[x1,x2]=meshgrid([-1:0.01:1],[-1:0.01:1]);
figure;
sgtitle('Geometry of the uncertain object for different values of uncertainty w')

subplot(2,2,1);hold on
w=0;
surf(x1,x2,eval(g),'FaceColor','blue','FaceAlpha',0.5,'EdgeColor','none','FaceLighting','phong');hold on; camlight; lighting gouraud
surf(x1,x2,0*ones(size(eval(g))),'FaceColor','white','FaceAlpha',1,'EdgeColor','none','FaceLighting','phong');hold on
ylabel('$x_2$','Interpreter','latex');xlabel('$x_1$','Interpreter','latex');set(gca,'fontsize',20)
text(-0.2,0.8,1,'$\omega=0$','Interpreter','latex','FontSize',25)

subplot(2,2,2);hold on
w=0.4;
surf(x1,x2,eval(g),'FaceColor','blue','FaceAlpha',0.5,'EdgeColor','none','FaceLighting','phong');hold on; camlight; lighting gouraud
surf(x1,x2,0*ones(size(eval(g))),'FaceColor','white','FaceAlpha',1,'EdgeColor','none','FaceLighting','phong');hold on
ylabel('$x_2$','Interpreter','latex');xlabel('$x_1$','Interpreter','latex');set(gca,'fontsize',20)
text(-0.2,0.8,1,'$\omega=0.4$','Interpreter','latex','FontSize',25)

subplot(2,2,3);hold on
w=0.6;
surf(x1,x2,eval(g),'FaceColor','blue','FaceAlpha',0.5,'EdgeColor','none','FaceLighting','phong');hold on; camlight; lighting gouraud
surf(x1,x2,0*ones(size(eval(g))),'FaceColor','white','FaceAlpha',1,'EdgeColor','none','FaceLighting','phong');hold on
ylabel('$x_2$','Interpreter','latex');xlabel('$x_1$','Interpreter','latex');set(gca,'fontsize',20)
text(-0.2,0.8,1,'$\omega=0.6$','Interpreter','latex','FontSize',25)

subplot(2,2,4);hold on
w=1;
surf(x1,x2,eval(g),'FaceColor','blue','FaceAlpha',0.5,'EdgeColor','none','FaceLighting','phong');hold on; camlight; lighting gouraud
surf(x1,x2,0*ones(size(eval(g))),'FaceColor','white','FaceAlpha',1,'EdgeColor','none','FaceLighting','phong');hold on
ylabel('$x_2$','Interpreter','latex');xlabel('$x_1$','Interpreter','latex');set(gca,'fontsize',20)
text(-0.2,0.8,1,'$\omega=1$','Interpreter','latex','FontSize',25)