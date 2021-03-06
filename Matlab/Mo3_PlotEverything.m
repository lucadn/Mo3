function [] = Mo3_PlotEverything(xStart,yStart,xPath,yPath,x_min, x_max, y_min, y_max,ObsList)
%Function plotting mobility patterns (2D case) generated by the Mo3 mobility model, as defined in
%L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
%future generation mobile wireless networks", submitted to IEEE Access
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

