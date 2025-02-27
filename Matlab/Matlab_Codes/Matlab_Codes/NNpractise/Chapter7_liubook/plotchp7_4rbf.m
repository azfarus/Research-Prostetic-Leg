%plot&_
close all; 
figure(1);
plot(out.tout,out.y.Data(:,1),'k', out.tout,out.y.Data(:,2),'r','linewidth',2);

% % plot(out.tout,out.qd.Data(:,1)
% xlabel('time(s)');ylabel('y'& 'ym');
% figure(2);
% plot(y(:,2),y(:,3),'k','linewidth',2);
% xlabel('x');ylabel('hj');
% hold on;
% plot(y(:,2),y(:,4),'k','linewidth',2);
% hold on;
% plot(y(:,2),y(:,5),'k','linewidth',2);
% hold on;
% plot(y(:,2),y(:,6),'k','linewidth',2);
% hold on;
% plot(y(:,2),y(:,7),'k','linewidth',2);out.yout{2}.Values.Data(:,1)