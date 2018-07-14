
function main
global thetapastmove;
global robotypast;
global robotxpast;
global goalx;
global goaly;
global offset;
world=[100,90]; %size of the world
start=[10,0];%robot starting location
goal=[90,85];%goal location
goalx=goal(1);
goaly=goal(2);
robotx=start(1);
roboty=start(2);
xdis=1;
ydis=1;
robotxpast=start(1);
robotypast=start(2);
robotsize=2;
offset=2;

%list of rectangle obstacles
rect1=  [6,10,27,4;
        39,10,25,4;
        70,10,24,4;
        39,180,24,4;
        70,180,24,4;
        6,180,24,4;
        150,180,24,4;
        110,180,24,4
        110,125,24,4
         
        70,90,4,50;
        20,90,4,50;
        35,110,20,4;
         150,10,24,4;
         130,20,24,4;
         60,20,24,4;
         20,20,24,4;
        110,10,24,4];
    

%list of circle obstacles
circ1= [30,40,10,10;
        67,55,7,7;
        15,55,7,7;
        110,55,7,7;
        150,55,7,7;
        180,55,7,7;
        110,85,7,7;
        150,85,7,7;
        180,85,7,7;
        110,143,7,7;
        150,135,7,7;
        180,135,7,7;
        40,40,3,3;
        55,40,3,3;
        70,40,3,3;
        85,40,3,3;
        100,40,3,3;
        115,40,3,3;
        130,40,3,3;
        145,40,3,3;
        45,70,3,3;
        60,70,3,3;
        75,70,3,3;
        90,70,3,3;
        105,70,3,3;
        120,70,3,3;
        135,70,3,3;
        90,110,3,3;
        90,127,3,3;
        38,90,3,3;
        52,90,3,3;
        105,110,3,3;
        120,110,3,3;
        150,110,3,3;
        170,160,3,3;
        105,160,3,3;
        120,160,3,3;
        150,160,3,3;
        170,160,3,3;
        15,160,3,3;
        30,160,3,3;
        45,160,3,3;
        60,160,3,3;
        75,160,3,3
        40,130,3,3
        58,130,3,3
        90,160,3,3];


%Obstacle array
global Obs;
Obs = ones(world(1),world(2));
%add rectangles
for i=1:length(rect1(:,1))
    addrec(rect1(i,:));
end
%add cirlces
for i=1:length(circ1(:,1))
    addcirc(circ1(i,:))
end

%start navigation loop
while (abs(xdis)>0)|| (abs(ydis)>0)
    
    
    %find where to go
    theta=atan2(ydis,xdis);%direction to point
    height=sin(theta);
    width=cos(theta);
    %set next point to clossest point to the destination
    if abs(height)>=((sqrt(2)/2)-.01)
        roboty=roboty+abs(sin(theta))/sin(theta);
    end
    if abs(width)>=((sqrt(2)/2)-.01)
        robotx=robotx+abs(cos(theta))/cos(theta);
    end
    
    %Check if obstical is in the new location 
    %in real life this could be done by a laser sensor or something
    if Obs(robotx,roboty)==0
        points=[0,0,2^17];
        %Check position 180 deg from current position and orientation
        [robotxtest1,robotytest1,dis1]=discalc(pi()/2);
        %set first check as best if it is open
        if Obs(robotxtest1,robotytest1)==1
            points=[robotxtest1,robotytest1,dis1];
        end
        %check postion 45 deg away
        [robotxtest2,robotytest2,dis2]=discalc(pi()/4);
        if Obs(robotxtest2,robotytest2)==1
            if dis2<points(3);
                %if its open and closer than the other point use it.
                points=[robotxtest2,robotytest2,dis2];
            end
            
        end
        %do the same checks for the angles from 180-(-180) including 0;
        [robotxtest3,robotytest3,dis3]=discalc(-pi()/4);
        if Obs(robotxtest3,robotytest3)==1
            if dis3<points(3);
            points=[robotxtest3,robotytest3,dis3];
            end
            
        end
        [robotxtest4,robotytest4,dis4]=discalc(-pi()/2);
        if Obs(robotxtest4,robotytest4)==1
            if dis4<points(3);
            points=[robotxtest4,robotytest4,dis4];
            end
            
        end
        [robotxtest5,robotytest5,dis5]=discalc(0);
        if Obs(robotxtest5,robotytest5)==1
            if dis5<points(3);
            points=[robotxtest5,robotytest5,dis5];
            end
            
        end
        
        %best point to chose is now points so set it to the next position    
        robotx=points(1);
        roboty=points(2);
    end
    
    
    %find orientation
    %find direction of past move    
    pastmovex=robotx-robotxpast;
    pastmovey=roboty-robotypast;
    %find theta of past move
    thetapastmove=atan2(pastmovey,pastmovex);
    %make points for the line indicating theta
    Arrowx=[robotx+robotsize/2,robotx+robotsize/2+2*cos(thetapastmove)];
    Arrowy=[roboty+robotsize/2,roboty+robotsize/2+2*sin(thetapastmove)];
    clf %start with new figure
    figure(1)
    axis([0,world(1),0,world(2)])
    
    %make robot postion vecotor for plotting a circle
    pos = [robotx roboty robotsize robotsize];
    %make goal postion vecotor for plotting a circle
    goalpos=[goal(1) goal(2) robotsize robotsize];
    %plot all rectangle obstacles
    for i=1:length(rect1(:,1))
     rectangle('Position',rect1(i,:),'FaceColor',[0 0 0])
    end
    %plot all circle obstacles
    for i=1:length(circ1(:,1))
        rectangle('Position',circ1(i,:),'FaceColor',[0 0 0],'Curvature',[1 1])
    end
    %plot goal
    rectangle('Position',goalpos,'FaceColor',[0 .5 0],'Curvature',[1 1])
    mytext= text(goal(1)-10,goal(2),'Goal')
    mytext.FontSize = 20;
    mytext.Color= 'r';
    %plot robot cirlce
    rectangle('Position',pos,'FaceColor',[.5 0 0],'Curvature',[1 1],'facecolor', 'r', 'edgecolor','r')
    hold on
    %plot robot orientation
    plot(Arrowx,Arrowy,'g','LineWidth',2)
    %delay so it doesnt happen instantly and plays like a movie
    pause(.1)
    
    %find new distance to goal this will end the loop when it gets to the
    %goal
    xdis=goal(1)-robotx;
    ydis=goal(2)-roboty;
    %set past distances
    robotxpast=robotx;
    robotypast=roboty;
end

end


%finds the distances to the goal for different points around the current
%location
function  [robotxtest1,robotytest1,dis1]= discalc(testangle)
global thetapastmove;
global robotypast;
global robotxpast;
global goalx;
global goaly;
testtheta1=thetapastmove+testangle;
        
        
        heighttest1=sin(testtheta1);
        widthtest1=cos(testtheta1);
        if abs(heighttest1)>=((sqrt(2)/2)-.01)
            robotytest1=robotypast+abs(sin(testtheta1))/sin(testtheta1);
        else
            robotytest1=robotypast;
        end
        if abs(widthtest1)>=((sqrt(2)/2)-.01)
            robotxtest1=robotxpast+abs(cos(testtheta1))/cos(testtheta1);
        else
            robotxtest1=robotxpast;
        end
        dis1=((goalx-robotxtest1)^2+(goaly-robotytest1)^2);
end
%adds a rectangle to the obstacle array
function addrec(X)
    global Obs;
    global offset;
    Obs(X(1)-offset:X(1)+X(3)+offset,X(2)-offset:X(2)+X(4)+offset)=0;
end
%adds circle to obstacle array
function addcirc(X)
    global Obs;
    global offset;
    for i=-X(3)-offset:X(3)+offset
        for j=-X(3):X(3);
         
            if (i^2+j^2)^.5<=(X(3)/2)+offset+3
                Obs(X(1)+i,X(2)+j)=0;
            end
               
        end
    end
   
end
