close all;
figure(1);
plot(out.tout,out.y.Data(:,1),'r',out.tout,out.y.Data(:,2),'k:','linewidth',2);
xlabel('time(s)');ylabel('y and ym');
legend('ideal signal','signal approximation');             
