function [] = Mo3_PlotEverything(xStart,yStart,xPath,yPath,x_min, x_max, y_min, y_max,ObsList)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
figure()
scatter(xStart,yStart,30,'filled')
hold on
for i=1:length(xStart)
 %plot(xPath(:,i),yPath(:,i),'Color',[0.929 0.694 0.125],'LineWidth',1.5)
 plot(xPath(:,i),yPath(:,i),'LineWidth',1.5)
end
hold on
Mo3_PlotObstacles(ObsList);
axis([x_min x_max y_min y_max])
grid on
xlabel('x [m]', 'FontSize',12)
ylabel('y [m]', 'FontSize',12)
set(gca,'FontSize',12)
end

