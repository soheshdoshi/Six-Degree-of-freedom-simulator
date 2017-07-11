function SiXDOF1
clc;
close all;
Theta=0:120:240;
DTheta=16.22;
BRad=141.78;
PRad=101.27;
ConnRod=218.2587;
LinkRod=20;
Height=200;
Pos=3;%x y z
PX=-10:10; %mm
PY=-10:10;%mm
PZ=-5:0.5:5;%mm
RX=-5:0.5:5;%degree
RY=-5:0.5:5;%degree
RZ=-5:0.5:5;%degree
BPC=zeros(7,3); %Base Plateform Coordinate
MPC=zeros(7,3); %Moving Plateform Coordinate
LPC=zeros(6,3); %Link Point Coordinate*/  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
BC=degtorad(sort([Theta  Theta+DTheta]))       %Base Circle
PC=degtorad(sort([Theta+60  Theta+DTheta+60])) %Plate Circle

%Calculate the Base Platform Point and Base Point Position
 %BPC=[BRad.*cos(BC)' BRad.*sin(BC)' zeros(1,6)']
BPC(1:6,:)=[ BRad.*cos(BC)'  BRad.*sin(BC)'  zeros(1,6)']
%Height of Moving Platform
MPC(1:6,:)=[ PRad.*cos(PC)' PRad.*sin(PC)'  Height*ones(1,6)']
% Height of Moving Platform
MPC(7,3)=Height;
%Display  the points in 3D
Point=[BPC;MPC]; %Base Points and Plateform Points 

figure,
xlim([-500 500]);
ylim([-500 500]);
zlim([-300 300]);

scatter3(Point(:,1),Point(:,2),Point(:,3),'filled');
% Draw Base Plate
DrawHexagon(BPC,1)
% Draw Moving Plate
DrawHexagon(MPC,1);
hold on;
%Cal Virtual Link
VirLink=LinkDist(BPC(1:7,:),MPC(1:7,:));
%Cal and Display Motor Plane
PlaneTheta=Theta+DTheta./2;
PlaneNormal=[ BRad*cos(degtorad(PlaneTheta))' BRad*sin(degtorad(PlaneTheta))' zeros(1,3)'];
hold on;
%scatter3(PlaneNormal(:,1),PlaneNormal(:,2),PlaneNormal(:,3));
MotorPlane=zeros(3,9);
 for i=1:3
    MotorPlane(i,:)=createPlane(BPC(i*2-1,:),PlaneNormal(i,:));
   drawPlane3d(MotorPlane(i,:));
 end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create the Theta with requied resolution 
Phi=0:0.25:90;
LC=degtorad(Phi);
%Cal Coordinate of Left Side Link
LPL=[ LinkRod.*cos(LC)' zeros(1,numel(Phi))' LinkRod.*sin(LC)' ];
%Cal Coordinate of Right Side Link
LPR=[ LinkRod.*cos(pi()-LC)' zeros(1,numel(pi()-Phi))' LinkRod.*sin(pi()-LC)' ];
scatter3(LPL(:,1),LPL(:,2),LPL(:,3));
scatter3(LPR(:,1),LPR(:,2),LPR(:,3));
[T1 T3 T5]=RotateZ1(LPL,degtorad(PlaneTheta+90));
[T0 T2 T4]=RotateZ1(LPR,degtorad(PlaneTheta+90));
 scatter3(T0(:,1),T0(:,2),T0(:,3));
 scatter3(T1(:,1),T1(:,2),T1(:,3));
 scatter3(T2(:,1),T2(:,2),T2(:,3));
TP0=Translate(T0,BPC(1,:));
TP1=Translate(T1,BPC(2,:)); 
TP2=Translate(T2,BPC(3,:));
TP3=Translate(T3,BPC(4,:)); 
TP4=Translate(T4,BPC(5,:));
TP5=Translate(T5,BPC(6,:)); 
TIndex=zeros(1,6);
% ConnRod0= sqrt(sum((MPC(6,:)-TP0(1,1:3)).^2))
% ConnRod1= sqrt(sum((MPC(1,:)-TP1(1,1:3)).^2))
% ConnRod2= sqrt(sum((MPC(2,:)-TP2(1,1:3)).^2))
% ConnRod3= sqrt(sum((MPC(3,:)-TP3(1,1:3)).^2))
% ConnRod4= sqrt(sum((MPC(4,:)-TP4(1,1:3)).^2))
% ConnRod5= sqrt(sum((MPC(5,:)-TP5(1,1:3)).^2))
% Cal min diff difference coordinate using Point on Motor Plane and Plane
% End Point 
[V0 TIndex(1,1)]=min( abs(sqrt(sum((repmat(MPC(6,:),length(Phi),1)-TP0(:,1:3))'.^2))-ConnRod));
[V1 TIndex(1,2)]=min( abs(sqrt(sum((repmat(MPC(1,:),length(Phi),1)-TP1(:,1:3))'.^2))-ConnRod));
[V2 TIndex(1,3)]=min( abs(sqrt(sum((repmat(MPC(2,:),length(Phi),1)-TP2(:,1:3))'.^2))-ConnRod));
[V3 TIndex(1,4)]=min( abs(sqrt(sum((repmat(MPC(3,:),length(Phi),1)-TP3(:,1:3))'.^2))-ConnRod));
[V4 TIndex(1,5)]=min( abs(sqrt(sum((repmat(MPC(4,:),length(Phi),1)-TP4(:,1:3))'.^2))-ConnRod));
[V5 TIndex(1,6)]=min( abs(sqrt(sum((repmat(MPC(5,:),length(Phi),1)-TP5(:,1:3))'.^2))-ConnRod));


JPC=[TP0(TIndex(1,1),1:3);TP1(TIndex(1,2),1:3);TP2(TIndex(1,3),1:3);TP3(TIndex(1,4),1:3);TP4(TIndex(1,5),1:3);TP5(TIndex(1,6),1:3)]
scatter3(JPC(:,1),JPC(:,2),JPC(:,3),'filled');
for i=1:6
   line([BPC(i,1) JPC(i,1)],[BPC(i,2) JPC(i,2)],[BPC(i,3) JPC(i,3)],'Color',[.1 .1 .1],'LineWidth',1);
%    sqrt(sum((BPC(i,:)-JPC(i,1:3)).^2))
   if(i==1)
       line([JPC(i,1) MPC(6,1)],[JPC(i,2) MPC(6,2)],[JPC(i,3) MPC(6,3)],'Color',[.2 .2 .2],'LineWidth',1);
       sqrt(sum((MPC(i+5,:)-JPC(i,1:3)).^2))
   else
       line([JPC(i,1) MPC(i-1,1)],[JPC(i,2) MPC(i-1,2)],[JPC(i,3) MPC(i-1,3)],'Color',[.2 .2 .2],'LineWidth',1);
       sqrt(sum((MPC(i-1,:)-JPC(i,1:3)).^2))
   end
end

%  TMPC=Transform(MPC,0,0,5,0,0,0,BPC,TP0,TP1,TP2,TP3,TP4,TP5,Phi,ConnRod);
hold on;
TMPC=Transform(MPC,0,0,0,0,0,0,BPC,TP0,TP1,TP2,TP3,TP4,TP5,Phi,ConnRod);
% hold on;
  TMPC=Transform(MPC,0,10,0,0,0,10,BPC,TP0,TP1,TP2,TP3,TP4,TP5,Phi,ConnRod);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Dist]=LinkDist(B,P)
    for i=1:7
        if(i==1)
             L = createLine3d(B(i,:),P(6,:));
             drawLine3d(L);
%            line([B(i,1) P(6,1)],[B(i,2) P(6,2)],[B(i,3) P(6,3)],'Color',[0 0 1],'LineWidth',1);
            
            Dist(i,1)=sqrt(sum((B(i,:)'-P(6,:)').^2)); 
        
        else
            if (i==7)
                L = createLine3d(B(i,:),P(i,:));
             drawLine3d(L);
%                 line([B(i,1) P(i,1)],[B(i,2) P(i,2)],[B(i,3) P(i,3)],'Color',[0 0 1],'LineWidth',1);
                Dist(i,1)=sqrt(sum((B(7,:)'-P(7,:)').^2)); 
            else
                L = createLine3d(B(i,:),P(i-1,:));
             drawLine3d(L);
                line([B(i,1) P(i-1,1)],[B(i,2) P(i-1,2)],[B(i,3) P(i-1,3)],'Color',[0 0 1],'LineWidth',1);
                Dist(i,1)=sqrt(sum((B(i,:)'-P(i-1,:)').^2));
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [TC]=Translate(T0,Base)

TC=([1 0 0 0; 0 1 0 0; 0 0 1 0; Base(1) Base(2) Base(3) 1]'*T0')';

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [RZ0,RZ1,RZ2]=RotateZ(LPoints,T)
    Temp=[LPoints  length(LPoints(1,length(LPoints)))];
    Mat0=[cos(T) sin(T) 0 0;-sin(T) cos(T) 0 0;0 0 1 0;0 0 0 1];
    Mat1=[cos(T) sin(T) 0 0;-sin(T) cos(T) 0 0;0 0 1 0;0 0 0 1];
    Mat2=[cos(T) sin(T) 0 0;-sin(T) cos(T) 0 0;0 0 1 0;0 0 0 1];
    RZ0=(Mat0'*Temp')';
    RZ1=(Mat1'*Temp')';
    RZ2=(Mat2'*Temp')';
end
function [RZ0,RZ1,RZ2]=RotateZ1(LPoints,T)
  Temp=[LPoints   ones(1,length(LPoints))'];
  Mat0=[cos(T(1)) sin(T(1)) 0 0;-sin(T(1)) cos(T(1)) 0 0;0 0 1 0;0 0 0 1];
  Mat1=[cos(T(2)) sin(T(2)) 0 0;-sin(T(2)) cos(T(2)) 0 0;0 0 1 0;0 0 0 1];
  Mat2=[cos(T(3)) sin(T(3)) 0 0;-sin(T(3)) cos(T(3)) 0 0;0 0 1 0;0 0 0 1];
  RZ0=(Mat0'*Temp')';
    RZ1=(Mat1'*Temp')';
    RZ2=(Mat2'*Temp')';
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function DrawHexagon(P,N)
    T=[P(1:size(P,1)-1,:); P(1,:)]
    line(T(:,1),T(:,2),T(:,3),'Color',[1 0 0],'LineWidth',N)%Base Points for Forming Circular Link Border "Red"
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function TMPC=Transform(MPC,Tx,Ty,Tz,Rx,Ry,Rz,BPC,TP0,TP1,TP2,TP3,TP4,TP5,Phi,ConnRod)
    RRx=degtorad(Rx);
    RRy=degtorad(Ry);
    RRz=degtorad(Rz);
    if sum([Rx Ry Rz])==0
        if(sum([Tx Ty Tz])==0)
            TMPC=MPC;
        else
            TMPC=MPC+repmat([Tx Ty Tz],7,1);
        end 
    else
        if(Rx~=0)
            Temp=[MPC   ones(1,length(MPC))'];
                 % Perform  Trasalation
                 Temp=Temp-repmat([MPC(7,:) 1],7,1)
                 % Perform Rotation 
                RxMat=[1    0        0          0;
                       0    cos(RRx) -sin(RRx)   0;
                       0    sin(RRx) cos(RRx)  0;
                       0        0           0     1];
                 TMPC=(RxMat'*Temp')';
                 % Perform Reverse Trasalation 
                 TMPC=TMPC+repmat([MPC(7,:) 1],7,1)
                 MPC=TMPC(:,1:3);
        end
        if(Ry~=0)
            Temp=[MPC   ones(1,length(MPC))'];
                 % Perform  Trasalation
                 Temp=Temp-repmat([MPC(7,:) 1],7,1)
                 % Perform Rotation 
                 RyMat=[cos(RRy)    0      sin(RRy) 0;
                        0           1      0        0;
                       -sin(RRy)    0      cos(RRy) 0;
                         0          0      0        1];
                 TMPC=(RyMat'*Temp')';
                 % Perform Reverse Trasalation 
                 TMPC=TMPC+repmat([MPC(7,:) 1],7,1)
                 MPC=TMPC(:,1:3);
        end
        if(Rz~=0)
                 Temp=[MPC   ones(1,length(MPC))'];
                 % Perform  Trasalation
                 Temp=Temp-repmat([MPC(7,:) 1],7,1)
                 % Perform Rotation 
                 RzMat=[cos(RRz) -sin(RRz)   0     0;
                        sin(RRz) cos(RRz)   0     0;
                       0        0           1     0;
                       0        0           0     1];
                 TMPC=(RzMat'*Temp')';
                 % Perform Reverse Trasalation 
                 TMPC=TMPC+repmat([MPC(7,:) 1],7,1)
            
        end
        
    end 
    scatter3(TMPC(:,1),TMPC(:,2),TMPC(:,3));
    DrawHexagon(TMPC,4);
    TIndex=zeros(1,6);
% ConnRod0= sqrt(sum((MPC(6,:)-TP0(1,1:3)).^2))
% ConnRod1= sqrt(sum((MPC(1,:)-TP1(1,1:3)).^2))
% ConnRod2= sqrt(sum((MPC(2,:)-TP2(1,1:3)).^2))
% ConnRod3= sqrt(sum((MPC(3,:)-TP3(1,1:3)).^2))
% ConnRod4= sqrt(sum((MPC(4,:)-TP4(1,1:3)).^2))
% ConnRod5= sqrt(sum((MPC(5,:)-TP5(1,1:3)).^2))
% Cal min diff difference coordinate using Point on Motor Plane and Plane
% End Point 
[V0 TIndex(1,1)]=min( abs(sqrt(sum((repmat(TMPC(6,1:3),length(Phi),1)-TP0(:,1:3))'.^2))-ConnRod));
[V1 TIndex(1,2)]=min( abs(sqrt(sum((repmat(TMPC(1,1:3),length(Phi),1)-TP1(:,1:3))'.^2))-ConnRod));
[V2 TIndex(1,3)]=min( abs(sqrt(sum((repmat(TMPC(2,1:3),length(Phi),1)-TP2(:,1:3))'.^2))-ConnRod));
[V3 TIndex(1,4)]=min( abs(sqrt(sum((repmat(TMPC(3,1:3),length(Phi),1)-TP3(:,1:3))'.^2))-ConnRod));
[V4 TIndex(1,5)]=min( abs(sqrt(sum((repmat(TMPC(4,1:3),length(Phi),1)-TP4(:,1:3))'.^2))-ConnRod));
[V5 TIndex(1,6)]=min( abs(sqrt(sum((repmat(TMPC(5,1:3),length(Phi),1)-TP5(:,1:3))'.^2))-ConnRod));


JPC=[TP0(TIndex(1,1),1:3);TP1(TIndex(1,2),1:3);TP2(TIndex(1,3),1:3);TP3(TIndex(1,4),1:3);TP4(TIndex(1,5),1:3);TP5(TIndex(1,6),1:3)]
scatter3(JPC(:,1),JPC(:,2),JPC(:,3),'filled');
for i=1:6
   line([BPC(i,1) JPC(i,1)],[BPC(i,2) JPC(i,2)],[BPC(i,3) JPC(i,3)],'Color',[0 0 0],'LineWidth',3);
%    sqrt(sum((BPC(i,:)-JPC(i,1:3)).^2))
   if(i==1)
       line([JPC(i,1) TMPC(6,1)],[JPC(i,2) TMPC(6,2)],[JPC(i,3) TMPC(6,3)],'Color',[0 0 0],'LineWidth',3);
       sqrt(sum((TMPC(i+5,1:3)-JPC(i,1:3)).^2))
   else
       line([JPC(i,1) TMPC(i-1,1)],[JPC(i,2) TMPC(i-1,2)],[JPC(i,3) TMPC(i-1,3)],'Color',[0 0 0],'LineWidth',3);
       sqrt(sum((TMPC(i-1,1:3)-JPC(i,1:3)).^2))
   end
end

TIndex
end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Planes=GetPlanes(B,P,RLink,RConn)
    Planes=zeros(7,4);%ax+by+cz+d=0
    for i=1:7
        if(i==1)
            line([B(i,1) P(6,1)],[B(i,2) P(6,2)],[B(i,3) P(6,3)],'Color',[0 0 1],'LineWidth',1);
            Dist(i,1)=sqrt(sum((B(i,:)'-P(6,:)').^2)); 
            Planes(i,1:3)=2*(B(i,1)- P(6,1:3));
            Planes(i,4)=(RLink^2-RConn^2)+(P(6,1)^2+P(6,2)^2+P(6,3)^2)-(B(i,1)^2+B(i,2)^2+B(i,3)^2);
            
        else
            if (i==7)
                line([B(i,1) P(i,1)],[B(i,2) P(i,2)],[B(i,3) P(i,3)],'Color',[0 0 1],'LineWidth',1);
                Dist(i,1)=sqrt(sum((B(7,:)'-P(7,:)').^2)); 
                  Planes(i,1:3)=2*(B(i,1)- P(i,1:3));
                Planes(i,4)=(RLink^2-RConn^2)+(P(i,1)^2+P(i,2)^2+P(i,3)^2)-(B(i,1)^2+B(i,2)^2+B(i,3)^2);
            else
                line([B(i,1) P(i-1,1)],[B(i,2) P(i-1,2)],[B(i,3) P(i-1,3)],'Color',[0 0 1],'LineWidth',1);
                Dist(i,1:3)=sqrt(sum((B(i,:)'-P(i-1,:)').^2));
                  Planes(i,1:3)=2.*(B(i,1)- P(i-1,1:3));
                Planes(i,4)=(RLink^2-RConn^2)+(P(i-1,1)^2+P(i-1,2)^2+P(i-1,3)^2)-(B(i,1)^2+B(i,2)^2+B(i,3)^2);
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% scatter3(TP1(1:25:end,1),TP1(1:25:end,2),TP1(1:25:end,3),'.');
% scatter3(TP2(1:25:end,1),TP2(1:25:end,2),TP2(1:25:end,3),'.');
% scatter3(TP3(1:25:end,1),TP3(1:25:end,2),TP3(1:25:end,3),'.');
% scatter3(TP4(1:25:end,1),TP4(1:25:end,2),TP4(1:25:end,3),'.');
% scatter3(TP5(1:25:end,1),TP5(1:25:end,2),TP5(1:25:end,3),'.');
% scatter3(TP2(:,1),TP2(:,2),TP2(:,3),'.');
% scatter3(TP3(:,1),TP3(:,2),TP3(:,3),'.');
% scatter3(TP4(:,1),TP4(:,2),TP4(:,3),'.');
% scatter3(TP5(:,1),TP5(:,2),TP5(:,3),'.');
%LinkPlane=GetPlanes(BPC(1:7,:),MPC(1:7,:),LinkRod,ConnRod);
% Plan
% LinkCircle=pi()/2+GetThetaFromABC_B(Dist(1:6),ConnRod,LinkRod);
% %LinkCircle=zeros(6,1);
% %LinkCircle=degtorad(LinkTheta);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LPoints=[ LinkRod.*cos(LinkCircle) zeros(1,numel(LinkCircle))' LinkRod.*sin(LinkCircle) ];
% hold on;
% scatter3(LPoints(:,1),LPoints(:,2),LPoints(:,3));
% for i=1:3
%     JPC(2*i-1,:)=RotateZ(LPoints(2*i-1,:),degtorad(PlaneTheta(i)+90));
%     JPC(2*i,:)=RotateZ(LPoints(2*i,:),degtorad(PlaneTheta(i)-90));
% end
% hold on;
% scatter3(JPC(:,1),JPC(:,2),JPC(:,3));
% for i=1:6
% JPC(i,:)=Translate(JPC(i,:),BPC(i,:)); 
% end
% scatter3(JPC(:,1),JPC(:,2),JPC(:,3),'filled');
% line(LPoints(:,1),LPoints(:,2),LPoints(:,3),'Color',[0 0 1],'LineWidth',1);
%     for i=1:6
%              line([BPC(i,1) JPC(i,1)],[BPC(i,2) JPC(i,2)],[BPC(i,3) JPC(i,3)],'Color',[.1 .1 .1],'LineWidth',1);
%        if(i==1)
%            line([JPC(i,1) MPC(6,1)],[JPC(i,2) MPC(6,2)],[JPC(i,3) MPC(6,3)],'Color',[.2 .2 .2],'LineWidth',1);
%        else
%            line([JPC(i,1) MPC(i-1,1)],[JPC(i,2) MPC(i-1,2)],[JPC(i,3) MPC(i-1,3)],'Color',[.2 .2 .2],'LineWidth',1);
%        end
%     end

% 
% 
% Phi=0:30:360;
% Rot=3;
% RComb=11;%-10 Degree to +10 Degree @ 2degree disp 
% PosLinkLen=zeros(PComb^Pos*RComb^Rot,13);%Index, Tx,Ty,Tz,Rx,Ry,Rz,L1,L2,L3,L4,L5,L6
% PLLIndex=1;
% 
% PlaneTheta=Theta+DTheta./2;
% PlaneNormal=[ BRad*cos(degtorad(PlaneTheta))' BRad*sin(degtorad(PlaneTheta))' zeros(1,3)'];
% 
% 
% 
% 
% 
% 
% LBPoint=[BPoints(1:6,:); BPoints(1,:)]; %Base Points for Forming Circular Link Border "Red"
% LPPoint=[PPoints(1:6,:); PPoints(1,:)]; %Platform Points for Forming Circular Link Border "Green"
% %Link(Virual Link ) End Points to Connect Platform and Base
% 
% %PlaneNormal Point from Origin for 2 Motor Pair Plane
% %scatter3(PlaneNormal(:,1),PlaneNormal(:,2),PlaneNormal(:,3));
% 
% line(LBPoint(:,1),LBPoint(:,2),LBPoint(:,3),'Color',[1 0 0],'LineWidth',4);%Base Points for Forming Circular Link Border "Red"
% line(LPPoint(:,1),LPPoint(:,2),LPPoint(:,3),'Color',[0 1 0],'LineWidth',3);%Platform Points for Forming Circular Link Border "Green"
% % For connecting Virtual Links 
% for i=1:7
%     line([BPoints(i,1) PPoints(i,1)],[BPoints(i,2) PPoints(i,2)],[BPoints(i,3) PPoints(i,3)],'Color',[0 0 1],'LineWidth',2);
% end
% For Connecting Virtual Link Between Center Point of Base and Plateform
% % i=7;
% % line([BPoints(i,1) PPoints(i,1)],[BPoints(i,2) PPoints(i,2)],[BPoints(i,3) PPoints(i,3)],'Color',[0 0 0],'LineWidth',4);
% 
% %%For Circle @ Borders 
% % LinkCircle=degtorad(Phi);
% % LPoints=[ LinkRad.*cos(LinkCircle)' zeros(1,numel(Phi))' LinkRad.*sin(LinkCircle)' ];
% %line(LPoints(:,1),LPoints(:,2),LPoints(:,3),'Color',[0 0 1],'LineWidth',3);% For Connecting Links Beetween Plateform and Base
% %line([Point(1,1)Point(1:7,2),Point(1:7,3),Point(8:14,1),Point(8:14,2),Point(8:14,3));
% Dist=LinkDist(BPoints(1:6,:),PPoints(1:6,:))
% PosLinkLen(PLLIndex,1)=PLLIndex;
% PosLinkLen(PLLIndex,8:13)=Dist;
% PLLIndex=PLLIndex+1;
% % for i=1:3
% % Plane=createPlane(BPoints(i*2-1,:),PlaneNormal(i,:));
% % drawPlane3d(Plane);
% % end
% 
% [T0 T1 T2]=RotateZ(LPoints,degtorad(PlaneTheta+90));
% 
% line(T0(:,1),T0(:,2),T0(:,3),'Color',[0.5 0 0],'LineWidth',3);
% line(T1(:,1),T1(:,2),T1(:,3),'Color',[0 0.5 0],'LineWidth',3);
% line(T2(:,1),T2(:,2),T2(:,3),'Color',[0 0 0.5],'LineWidth',3);
% 
% TP0=Translate(T0,BPoints(1,:)); 
% TP1=Translate(T0,BPoints(2,:)); 
% TP2=Translate(T1,BPoints(3,:));
% TP3=Translate(T1,BPoints(4,:));
% TP4=Translate(T2,BPoints(5,:));
% TP5=Translate(T2,BPoints(6,:));
% scatter3(TP0(:,1),TP0(:,2),TP0(:,3),'Red','filled');
% scatter3(TP1(:,1),TP1(:,2),TP1(:,3),'Blue','filled');
% line(TP2(:,1),TP2(:,2),TP2(:,3),'Color',[0 0 0],'LineWidth',3);
% line(TP3(:,1),TP3(:,2),TP3(:,3),'Color',[0 0 0],'LineWidth',3);
% line(TP4(:,1),TP4(:,2),TP4(:,3),'Color',[0 0 0],'LineWidth',3);
% line(TP5(:,1),TP5(:,2),TP5(:,3),'Color',[0 0 0],'LineWidth',3);
% 
% % end
% end
