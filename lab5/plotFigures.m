function plotFigures(jfinal,kfinal,Ts_int,Ts_ext,simulationTime,motorVoltages,traj,xUni,vUni);

% motor voltages
figure();
plot([1:jfinal*kfinal]*Ts_int,motorVoltages(1,:),'b','LineWidth',1.5); hold; grid;
plot([1:jfinal*kfinal]*Ts_int,motorVoltages(2,:),'m--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Voltages (V)');
legend('Right motor','Left motor');
xlim([0 simulationTime]);

% x X y
figure();
plot(xUni(1,:),xUni(2,:),'b','LineWidth',1.5); grid; hold;
plot(traj.x(1,1:kfinal+1),traj.x(3,1:kfinal+1),'r--','LineWidth',1);
xlabel('x(m)'); ylabel('y(m)');
vert_obs{1} = [1.5 3;3 3;3 0;1.5 0];
fill(vert_obs{1}(:,1),vert_obs{1}(:,2),'k');
vert_tar{1} = [4 5;5 5;5 4;4 4];
fill(vert_tar{1}(:,1),vert_tar{1}(:,2),'w');
ylabel('y(m)'); xlabel('x(m)');

% position X time
figure();
subplot(1,2,1)
plot([1:jfinal*kfinal+1]*Ts_int,xUni(1,:),'b','LineWidth',1.5); grid; hold;
plot([1:kfinal+1]*Ts_ext,traj.x(1,1:kfinal+1),'r--','LineWidth',1);
legend('Pos.','Ref.');
ylabel('x(m)'); xlabel('Time (s)');
subplot(1,2,2)
plot([1:jfinal*kfinal+1]*Ts_int,xUni(2,:),'b','LineWidth',1.5); grid; hold;
plot([1:kfinal+1]*Ts_ext,traj.x(3,1:kfinal+1),'r--','LineWidth',1);
legend('Pos.','Ref.');
ylabel('y(m)'); xlabel('Time (s)');


% velocidades X tempo
figure();
subplot(1,2,1)
plot([1:kfinal+1]*Ts_ext,vUni(1,:),'b','LineWidth',1.5); grid; hold;
plot([1:kfinal+1]*Ts_ext,traj.x(2,1:kfinal+1),'r--','LineWidth',1);
legend('Vel.','Ref.');
ylabel('v_x(m/s)'); xlabel('Time (s)');
subplot(1,2,2)
plot([1:kfinal+1]*Ts_ext,vUni(2,:),'b','LineWidth',1.5); grid; hold;
plot([1:kfinal+1]*Ts_ext,traj.x(4,1:kfinal+1),'r--','LineWidth',1);
legend('Vel.','Ref.');
ylabel('v_x(m/s)'); xlabel('Time (s)');

% Orientação
figure();
plot([1:jfinal*kfinal+1]*Ts_int,xUni(3,:)*180/pi,'b','LineWidth',1.5); grid;
ylabel('Orienatation(degrees)'); xlabel('Time (s)');
end

