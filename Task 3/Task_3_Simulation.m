clear
close all

%---------------- Read Me----------------------------------%
% run is the function that takes x y z and outputs id values and sets gamma
% to 270(straight down)
% flip takes x y z and outputs id values and sets gamma to 0 (straight
% ahead)
% x and y are given as coordinates
% z is given as distance
%---------------- X Y Z values----------------------------------%
% x1=0; %x as coordinate
% x2=2;
% x3=-3.5;
% y1=4;
% y2=0;%y as coordinate
% z1=0.058;    %z as distance
%---------------- Grid Set up----------------------------------%


grid on
view(15,15)
axis equal
title("Draw a square in 3 planes")
xlim([-0.3 0.3])
ylim([-0.3 0.3])
zlim([-0.3 0.4])
xlabel(['x']);
ylabel(['y']);
zlabel(['z']);

%---------------- ID values----------------------------------%
Z_DIST      = 0.0763;
GAMMA_ANGLE = 365;
PEN_START   = [7.4,7.4,0.0945,GAMMA_ANGLE];
A           = [-7,4,Z_DIST,GAMMA_ANGLE];
B           = [-7,8,Z_DIST,GAMMA_ANGLE];
C           = [-5,6,Z_DIST,GAMMA_ANGLE];
D           = [-7,6,Z_DIST,GAMMA_ANGLE];
PEN_END     = [7.15,7.15,0.09,GAMMA_ANGLE];


%% ------ Angles For Drawing -----%%

radius = 1;
angle = 270;

id_array1 = circle(D(1), D(2), Z_DIST, radius, angle,GAMMA_ANGLE);
id_array2 = straight_horizontal_line(D(1),C(1),C(2),Z_DIST,GAMMA_ANGLE);
id_array3 = diagonal_line(C(1),B(1),C(2),B(2),Z_DIST,GAMMA_ANGLE);
id_array4 = straight_vertical_line(B(1),B(2),A(2),Z_DIST,GAMMA_ANGLE);




angles = [id_array1;id_array2;id_array3;id_array4];


 
 

% [id11f,id12f,id13f,id14f]=flip(x,y,z);
%---------------- Create DH-Matrix----------------------------------%
function Ti = createDHMatrix(alpha, a, d, thetha)
    Ti = [cosd(thetha) -sind(thetha) 0 a;
          sind(thetha)*cosd(alpha) cosd(thetha)*cosd(alpha) -sind(alpha) -sind(alpha)*d;  
          sind(thetha)*sind(alpha) cosd(thetha)*sind(alpha) cosd(alpha) cosd(alpha)*d;
          0 0 0 1 ];
    
end
%---------------- Forward Kinematics----------------------------------%
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
%---------------- Plot forward Kinematics----------------------------------%
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
    pl0.LineWidth = 5;
    pl1.LineWidth = 5;
    pl2.LineWidth = 5;
    pl3.LineWidth = 5;
    pl4.LineWidth = 5;
    
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
%---------------- Inverse Kinematics----------------------------------%
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

%---------------- Delete Robot----------------------------------%
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






  
%---------------- Flip function(gamma equals 0)----------------------------------%
function [id11,id12,id13,id14]=flip(x,y,z)
gamma=0;
[id11,id12,id13,id14]=kinematics2(x,y,z,gamma);
end

%---------------- Run function(gamma equals 270/straight down)----------------------------------%
function [id11,id12,id13,id14]=run(x,y,z)
gamma=350;
[id11,id12,id13,id14]=kinematics2(x,y,z,gamma);
end

function [id11,id12,id13,id14]=kinematics2(x_coord,y_coord,z,gamma)
    [x,y] = coord_to_dist(x_coord,y_coord);
    [spin,theta_one,theta_two,theta_three]=doInverseKinematics(x,y,z,gamma);
    [T01, T02, T03, T04, T05] = doForwardKinematics(spin,theta_one(2),theta_two(2),theta_three(2));
    hold on
    [line,frame1,frame2,frame3,frame4,frame5]= plotForward(T01, T02, T03, T04, T05);
    p=plot3(T05(1,4),T05(2,4),T05(3,4),'.');
    p(1).MarkerSize = 10;
    pause(0.1)
    deleterobo(line,frame1,frame2,frame3,frame4,frame5)
%---------------- Offsets to map model to real----------------------------------%
offset1=10.62;
offset2 = 10.62;

spin=spin-90;
theta_one(2)= 270-theta_one(2)-offset1;
theta_two(2) = 90-theta_two(2)+offset2;
theta_three(2) = 180-theta_three(2);

%---------------- Positive angles----------------------------------%
if spin<0
    spin=spin+360;
end
if theta_one(2)<0
    theta_one(2);
    theta_one=theta_one+360;
end
if theta_two(2)<0
    theta_two(2);
    theta_two=theta_two+360;
end
if theta_three(2)<0
    theta_three(2);
    theta_three=theta_three+360;
end

id11=(((spin)/360)*4096);
id12=(theta_one(2)/360)*4096;
id13=(theta_two(2)/360)*4096;
id14=(((theta_three(2))/360)*4096);

end
%---------------- Map coordinates to distances----------------------------------%
function [x_dist,y_dist]=coord_to_dist(x_coord,y_coord)
x_dist = -0.025*x_coord;
y_dist = -0.025*y_coord;
end

function id_array = straight_horizontal_line(x1,x2,y1,z,gamma)
    xdiff=abs(x1-x2);
    
    n=xdiff*5;
    xvals = linspace(x1,x2,n);
    [id11, id12, id13, id14] = kinematics2(xvals(1),y1,z,gamma);
    id_array =  [id11 id12 id13 id14];
    i=2;

    while i<=n
        [b1, b2, b3, b4] =kinematics2(xvals(i),y1,z,gamma);
        b = [b1 b2 b3 b4];
        i=i+1;
        id_array = [id_array;b];
    end
end

function id_array = straight_vertical_line(x1,y1,y2,z,gamma)
    ydiff= abs(y1-y2);
    n=ydiff*5;
    yvals = linspace(y1,y2,n);
    [id11, id12, id13, id14] = kinematics2(x1,yvals(1),z,gamma);
    id_array =  [id11 id12 id13 id14];
    i=2;
   
    while i<=n
        [b1, b2, b3, b4] = kinematics2(x1,yvals(i),z,gamma);
        b = [b1 b2 b3 b4];
        i=i+1;
        id_array = [id_array;b];
    end  
end

function id_array = diagonal_line(x1,x2,y1,y2,z,gamma)
    ydiff= abs(y1-y2);
    n=ydiff*5;
    xvals=linspace(x1,x2,n);
    yvals=linspace(y1,y2,n);
    [id11, id12, id13, id14] = kinematics2(xvals(1),yvals(1),z,gamma);
    id_array =  [id11 id12 id13 id14];
    i=2;
    k=2;
    
    while i<=n
        [b1, b2, b3, b4] = kinematics2(xvals(i),yvals(k),z,gamma);
        b = [b1 b2 b3 b4];
        i=i+1;
        k=k+1;
        id_array = [id_array;b];
     
    end
end

function id_array = circle(x1,y1,z,r,angle,gamma)
    n=(angle/90)*20;
    theta = linspace(0, angle*pi/180, n); % Define angle range in reverse order
    
    if x1<0
%         xvals = -r*cos(theta)+x1+r; % Define x coordinates with offset and sign change
    xvals = -r*cos(theta)+x1;
    end
    
    if x1>0
%        xvals = r*cos(theta)+x1-r; % Define x coordinates with offset and sign change
    xvals = r*cos(theta)+x1;
    end
   
    yvals = r*sin(theta)+y1+r; % Define y coordinates
    [id11, id12, id13, id14] = kinematics2(xvals(1),yvals(1),z,gamma);
    id_array =  [id11 id12 id13 id14];
    i=2;
    k=2;
   
    while i<=n
        [b1, b2, b3, b4] = kinematics2(xvals(i),yvals(k),z,gamma);
        b = [b1 b2 b3 b4];
        i=i+1;
        k=k+1;
        id_array = [id_array;b];
    end  
end

function id_array=drawshape(x1,x2,x3,y1,y2,z1)
id_array1=straight_horizontal_line(x1,x2,y1,z1);
id_array2=straight_vertical_line(x2,y1,y2,z1);
id_array3=diagonal_line(x2,x3,y2,y1,z1);
id_array4=circle(x3,y1,z1,1,90);

id_array = [id_array1;id_array2;id_array3;id_array4;];

end




