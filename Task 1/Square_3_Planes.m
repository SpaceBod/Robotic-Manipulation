clear
close all

x=0.05;
y=0.05;
z=0.05;
speed=0.1;
grid on
view(15,15)
title("Draw a square in 3 planes")
xlim([-0.3 0.3])
ylim([-0.3 0.3])
zlim([0 0.4])
xlabel(['x']);
ylabel(['y']);
zlabel(['z']);
pause(5)
drawsquarexy('r',speed,0.03,0.03,0.03)
drawsquarezy('b',speed,-0.03,0.03,0.03)
drawsquarezx('g',speed,-0.10,0.06,0.06)

function Ti = createDHMatrix(alpha, a, d, thetha)
    Ti = [cosd(thetha) -sind(thetha) 0 a;
          sind(thetha)*cosd(alpha) cosd(thetha)*cosd(alpha) -sind(alpha) -sind(alpha)*d;  
          sind(thetha)*sind(alpha) cosd(thetha)*sind(alpha) cosd(alpha) cosd(alpha)*d;
          0 0 0 1 ];
    
end

function [T01, T02, T03,T04, T05] = doForwardKinematics(spin, rot_1, rot_2, rot_3)
    % DH table
    thetha1 = spin;
    thetha2 = rot_1;
    thetha3 = rot_2;
    thetha4 = rot_3;
    thetha5 = 0;
   
    % angle between axis, measured along X
    alpha = [0 90 0 0 0];  

    % distance between axis measured along X
    a = [0 0 0.130 0.124 0.126 ]; 

    % distance between links, measured along Z
    d = [0.077 0 0 0 0]; 
    
    % angle between links, measured about Z
    thetha =[thetha1 thetha2 thetha3 thetha4 thetha5]; 

    
    
    T1 = createDHMatrix(alpha(1),a(1),d(1),thetha(1));%T_0_1
    T2 = createDHMatrix(alpha(2),a(2),d(2),thetha(2));%T_1_2
    T3 = createDHMatrix(alpha(3),a(3),d(3),thetha(3));
    T4 = createDHMatrix(alpha(4),a(4),d(4),thetha(4));
    T5 = createDHMatrix(alpha(5),a(5),d(5),thetha(5));
    
    T01=T1;
    T02=T1*T2;
    T03=T1*T2*T3;
    T04=T1*T2*T3*T4;
    T05=T1*T2*T3*T4*T5;
    
end

function [lines,frame1,frame2,frame3,frame4,frame5]=plotForward(T01, T02, T03, T04, T05)
    origin = [0 0 0];
    base = [T01(1,4) T01(2,4) T01(3,4)];
    shoulder = [T02(1,4) T02(2,4) T02(3,4)];
    elbow = [T03(1,4) T03(2,4) T03(3,4)];
    arm = [T04(1,4) T04(2,4) T04(3,4)];
    wrist = [T05(1,4) T05(2,4) T05(3,4)];

    
    pl0=line([origin(1) base(1)], [origin(2) base(2)], [origin(3) base(3)],'Color','k');
    pl1=line([base(1) shoulder(1)], [base(2) shoulder(2)], [base(3) shoulder(3)],'Color','k');
    pl2=line([shoulder(1) elbow(1)], [shoulder(2) elbow(2)], [shoulder(3) elbow(3)],'Color','k');
    pl3=line([elbow(1) arm(1)], [elbow(2) arm(2)], [elbow(3) arm(3)],'Color','k');
    pl4=line([arm(1) wrist(1)], [arm(2) wrist(2)], [arm(3) wrist(3)],'Color','k');
    pl0.LineWidth = 3;
    pl1.LineWidth = 3;
    pl2.LineWidth = 3;
    pl3.LineWidth = 3;
    pl4.LineWidth = 3;
    
    lines=[pl0,pl1,pl2,pl3,pl4];
    
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
    
end

function [spin_angle, theta_one, theta_two, theta_three] = doInverseKinematics(target_x, target_y, target_z,gamma)
l_one = 0.130;
l_two = 0.124;
l_three = 0.126;
%---------------- Spin Angle----------------------------------%
spin_angle = atan2d(target_y,target_x);

%---------------- Wrist Coordinates----------------------------------%
r_end = sqrt((target_x^2)+ (target_y^2));
r = r_end-cosd(gamma)*l_three;

x_wrist = cosd(spin_angle)*r;
y_wrist = sind(spin_angle)*r;
z_diff = l_three*sind(gamma);

z_wrist = target_z-z_diff;

target_plane = sqrt((x_wrist^2)+ (y_wrist^2));
z1 = 0.077;
pos_z = z_wrist-z1;

%---------------- Theta two----------------------------------%
cos_theta_two = (((target_plane^2)+(pos_z^2)-(l_one^2)-(l_two^2))/(2*l_one*l_two));
sin_theta_two(1) = sqrt(1-(cos_theta_two^2));
sin_theta_two(2) = -sqrt(1-(cos_theta_two^2));

theta_two(1)=atan2d(sin_theta_two(1),cos_theta_two);
theta_two(2)=atan2d(sin_theta_two(2),cos_theta_two);

%---------------- Theta one----------------------------------%
k_one = l_one +l_two*cos_theta_two;
k_two(1) = l_two*sin_theta_two(1);
k_two(2) = l_two*sin_theta_two(2);

theta_one(1) = atan2d(pos_z,target_plane)-atan2d(k_two(1),k_one);
theta_one(2) = atan2d(pos_z,target_plane)-atan2d(k_two(2),k_one);

%---------------- Theta Three----------------------------------%
theta_three(1) = gamma-(theta_two(1)+theta_one(1));
theta_three(2) = gamma-(theta_two(2)+theta_one(2));

end



function deleterobo_frame(frame)
    delete(frame(1))
    delete(frame(2))
    delete(frame(3))
end
function deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    delete(line)
    deleterobo_frame(frame1)
    deleterobo_frame(frame2)
    deleterobo_frame(frame3)
    deleterobo_frame(frame4)
    deleterobo_frame(frame5)   
end
function [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,gamma)
    [spin,theta_one,theta_two,theta_three]=doInverseKinematics(x,y,z,gamma);
    [T01, T02, T03, T04, T05] = doForwardKinematics(spin,theta_one(2),theta_two(2),theta_three(2));
    hold on
    [line,frame1,frame2,frame3,frame4,frame5]=plotForward(T01, T02, T03, T04, T05);
%     p=plot3(x,y,z,colour);
%     p(1).MarkerSize = 10;
end
function drawline(colour,x1,x,y1,y,z1,z)
line([x1 x], [y1 y], [z1 z],'Color',colour,'LineWidth',3);
end

function drawsquarexy(colour,speed,x,y,z)
    x1=x;
    y1=y;
    z1=z;
    i=0;
    while i <10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    x1=x;
    y1=y;
    x=x+0.01;
    deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
    end
   i=0;
   while i <10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    x1=x;
    y1=y;
    y=y+0.01;
    deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
   end
   i=0;
   while i <10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    x1=x;
    y1=y;
    x=x-0.01;
   deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
   end
   i=0;
   while i <=10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    x1=x;
    y1=y;
    y=y-0.01;
    deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
   end

end
function drawsquarezy(colour,speed,x,y,z)

    x1=x;
    y1=y;
    z1=z;
    i=0;
    while i <10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    z1=z;
    y1=y;
    z=z+0.01;
    deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
    end
   i=0;
   while i <10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    z1=z;
    y1=y;
    y=y+0.01;
    deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
   end
   i=0;
   while i <10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    z1=z;
    y1=y;
    z=z-0.01;
   deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
   end
   i=0;
   while i <=10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    z1=z;
    y1=y;
    y=y-0.01;
    deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
   end

end
function drawsquarezx(colour,speed,x,y,z)
    x1=x;
    y1=y;
    z1=z;
    i=0;
    while i <10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    z1=z;
    x1=x;
    z=z+0.01;
    deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
    end
   i=0;
   while i <10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    z1=z;
    x1=x;
    x=x+0.01;
    deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
   end
   i=0;
   while i <10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    z1=z;
    x1=x;
    z=z-0.01;
   deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
   end
   i=0;
   while i <=10
    [line,frame1,frame2,frame3,frame4,frame5]=ik_to_fk(x,y,z,270);
    drawline(colour,x1,x,y1,y,z1,z)
    pause(speed)
    z1=z;
    x1=x;
    x=x-0.01;
    deleterobo(line,frame1,frame2,frame3,frame4,frame5)
    i=i+1;
   end


end

