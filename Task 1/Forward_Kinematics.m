clear
close all

%---------------- DH table----------------------------------%
thetha1=0;
thetha2=0;
thetha3=0;
thetha4=0;
thetha5=0;


alpha = [0 90 0 0 0 0]; %angle between axis, measured along X
a = [0 0 0.130 0.124 0.126 0 ];%distance between axis measured along X 
d = [0.077 0 0 0 0 0]; %distance between links, measured along Z
thetha =[thetha1 thetha2 thetha3 thetha4 thetha5]; %angle between links, measured about Z
hold on

T1 = createDH(alpha(1),a(1),d(1),thetha(1));%T_0_1
T2 = createDH(alpha(2),a(2),d(2),thetha(2));%T_1_2
T3 = createDH(alpha(3),a(3),d(3),thetha(3));
T4 = createDH(alpha(4),a(4),d(4),thetha(4));
T5 = createDH(alpha(5),a(5),d(5),thetha(5));
%---------------- DH Matrices----------------------------------%
T01=T1;
T02=T1*T2;
T03=T1*T2*T3;
T04=T1*T2*T3*T4;
T05=T1*T2*T3*T4*T5;

%---------------- Position assignment----------------------------------%
origin = [0 0 0];
base = [T01(1,4) T01(2,4) T01(3,4)];
shoulder = [T02(1,4) T02(2,4) T02(3,4)];
elbow = [T03(1,4) T03(2,4) T03(3,4)];
arm = [T04(1,4) T04(2,4) T04(3,4)];
wrist = [T05(1,4) T05(2,4) T05(3,4)];


%----------------Plot Robot Lines----------------------------------%
pl0=line([origin(1) base(1)], [origin(2) base(2)], [origin(3) base(3)],'Color','k');
pl1=line([base(1) shoulder(1)], [base(2) shoulder(2)], [base(3) shoulder(3)],'Color','k');
pl2=line([shoulder(1) elbow(1)], [shoulder(2) elbow(2)], [shoulder(3) elbow(3)],'Color','k');
pl3=line([elbow(1) arm(1)], [elbow(2) arm(2)], [elbow(3) arm(3)],'Color','k');
pl4=line([arm(1) wrist(1)], [arm(2) wrist(2)], [arm(3) wrist(3)],'Color','k');
pl0.LineWidth = 5;
pl1.LineWidth = 5;
pl2.LineWidth = 5;
pl3.LineWidth = 5;
pl4.LineWidth = 5;

%----------------Robot Frames----------------------------------%
% quiver3(T01(1,4),T01(2,4),T01(3,4),T01(1,1),T01(2,1),T01(3,1),0.04,'r', 'LineWidth', 2);
% quiver3(T01(1,4),T01(2,4),T01(3,4),T01(1,2),T01(2,2),T01(3,2),0.04,'g', 'LineWidth', 2);
% quiver3(T01(1,4),T01(2,4),T01(3,4),T01(1,3),T01(2,3),T01(3,3),0.04,'b', 'LineWidth', 2);

f1x=quiver3(0,0,0,T01(1,1),T01(2,1),T01(3,1),0.04,'r', 'LineWidth', 2);
f1y=quiver3(0,0,0,T01(1,2),T01(2,2),T01(3,2),0.04,'g', 'LineWidth', 2);
f1z=quiver3(0,0,0,T01(1,3),T01(2,3),T01(3,3),0.04,'b', 'LineWidth', 2);



f2x=quiver3(T02(1,4),T02(2,4),T02(3,4),T02(1,1),T02(2,1),T02(3,1),0.04,'r', 'LineWidth', 2);
f2y=quiver3(T02(1,4),T02(2,4),T02(3,4),T02(1,2),T02(2,2),T02(3,2),0.04,'g', 'LineWidth', 2);
f2z=quiver3(T02(1,4),T02(2,4),T02(3,4),T02(1,3),T02(2,3),T02(3,3),0.04,'b', 'LineWidth', 2);

f3x=quiver3(T03(1,4),T03(2,4),T03(3,4),T03(1,1),T03(2,1),T03(3,1),0.04,'r', 'LineWidth', 2);
f3y=quiver3(T03(1,4),T03(2,4),T03(3,4),T03(1,2),T03(2,2),T03(3,2),0.04,'g', 'LineWidth', 2);
f3z=quiver3(T03(1,4),T03(2,4),T03(3,4),T03(1,3),T03(2,3),T03(3,3),0.04,'b', 'LineWidth', 2);

f4x=quiver3(T04(1,4),T04(2,4),T04(3,4),T04(1,1),T04(2,1),T04(3,1),0.04,'r', 'LineWidth', 2);
f4y=quiver3(T04(1,4),T04(2,4),T04(3,4),T04(1,2),T04(2,2),T04(3,2),0.04,'g', 'LineWidth', 2);
f4z=quiver3(T04(1,4),T04(2,4),T04(3,4),T04(1,3),T04(2,3),T04(3,3),0.04,'b', 'LineWidth', 2);

f5x=quiver3(T05(1,4),T05(2,4),T05(3,4),T05(1,1),T05(2,1),T05(3,1),0.04,'r', 'LineWidth', 2);
f5y=quiver3(T05(1,4),T05(2,4),T05(3,4),T05(1,2),T05(2,2),T05(3,2),0.04,'g', 'LineWidth', 2);
f5z=quiver3(T05(1,4),T05(2,4),T05(3,4),T05(1,3),T05(2,3),T05(3,3),0.04,'b', 'LineWidth', 2); 

frame1 =[f1x,f1y,f1z];
frame2 = [f2x,f2y,f2z];
frame3 = [f3x,f3y,f3z];
frame4 = [f4x,f4y,f4z];
frame5 = [f5x,f5y,f5z];

%----------------End effector coordinates----------------------------------%
wrist(1)
wrist(2)
wrist(3)

%----------------Grid Set up----------------------------------%
xlim([-0.5 0.5])
ylim([-0.5 0.5])
zlim([0 0.5])
xlabel(['x']);
ylabel(['y']);
zlabel(['z']);

title(["Forward Kinematics:","theta4= ",num2str(thetha4),"theta3= ", num2str(thetha3), "theta2=", num2str(thetha2), "theta1=", num2str(thetha1)])
grid on
view(15,15)
















% DH matrix%
function Ti=createDH(alpha, a, d, thetha)
    Ti = [cosd(thetha) -sind(thetha) 0 a;
          sind(thetha)*cosd(alpha) cosd(thetha)*cosd(alpha) -sind(alpha) -sind(alpha)*d;  
          sind(thetha)*sind(alpha) cosd(thetha)*sind(alpha) cosd(alpha) cosd(alpha)*d;
          0 0 0 1 ];
    
end







