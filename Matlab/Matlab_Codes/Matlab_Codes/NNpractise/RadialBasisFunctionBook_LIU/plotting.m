close all; 

figure(1);
plot(out.tout,out.y.Data(:,1),'k','linewidth',2)
xlabel('time(s)');ylabel('y');
figure(2);
plot(out.y.Data(:,2),out.y.Data(:,3),'k','linewidth',2);
xlabel('x');ylabel('hj');
hold on;
plot(out.y.Data(:,2),out.y.Data(:,4),'k','linewidth',2);
hold on;
plot(out.y.Data(:,2),out.y.Data(:,5),'k','linewidth',2);
hold on;
plot(out.y.Data(:,2),out.y.Data(:,6),'k','linewidth',2);
hold on;
plot(out.y.Data(:,2),out.y.Data(:,7),'k','linewidth',2);