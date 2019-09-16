function [ACCX,ACCY,ACCROTZ] = modelDx(VELX,VELY,VELROTZ,BETA,AB,TV, param)

% BETA : Lenkwinkel (control)
% AB : acceleration of hinterachse (control)
% TV : torque vectoring
% AB-TV rechte achse
% AB+TV linke achse

    %param = [B1,C1,D1,B2,C2,D2,Ic]; 
    % Parameters for the Pacejka's Formula
    B1 = param(1);
    C1 = param(2);
    D1 = param(3);
    B2 = param(4);
    C2 = param(5);
    D2 = param(6);
    
    %Moment of inertia
    Ic = param(7); 
    
    %maxA = param(8);
    % Pacejka's magic formula
    magic = @(s,B,C,D)D.*sin(C.*atan(B.*s));
    
    % regultaion factor R_s
    reg = 0.5;
    
    % function that expresses the remaining capacity for sidewards grip
    capfactor = @(taccx)(1-satfun((taccx/D2)^2))^(1/2);
    
    % slip definition
    simpleslip = @(VELY,VELX,taccx)-(1/capfactor(taccx))*VELY/(VELX+reg);
    %simpleslip = @(VELY,VELX,taccx)-VELY/(VELX+reg);
    
    % Equations for the approximated lateral force: Map (fx,vy) --> (fy)
    simplediraccy = @(VELY,VELX,taccx)magic(simpleslip(VELY,VELX,taccx),B2,C2,D2);
    simpleaccy = @(VELY,VELX,taccx)capfactor(taccx)*simplediraccy(VELY,VELX,taccx);
    
    %acclim = @(VELY,VELX, taccx)(VELX^2+VELY^2)*taccx^2-VELX^2*maxA^2;
    
    % Equation for the lateral force in tire frame
    simplefaccy = @(VELY,VELX)magic(-VELY/(VELX+reg),B1,C1,D1);
    %simpleaccy = @(VELY,VELX,taccx)magic(-VELY/(VELX+reg),B2,C2,D2);
    
    % go-kart length between axles
    l = 1.19;
    
    % distance from the front axle to the center of mass
    l1 = 0.73;
    
    % distance from the back axle to the center of mass
    l2 = l-l1;
    
    % normal forces ( g is in D)
    f1n = l2/l;
    f2n = l1/l;
    
    % Rotation Matrix
    rotmat = @(beta)[cos(beta),sin(beta);-sin(beta),cos(beta)];
    
    % longitudinal and lateral velocity in the tire frame
    vel1 = rotmat(BETA)*[VELX;VELY+l1*VELROTZ];
    
    % Lateral force (acceleration) in tire frame
    f1y = simplefaccy(vel1(2),vel1(1));
    F1 = rotmat(-BETA)*[0;f1y]*f1n;
    
    % Longitudinal force in the go-kart frame (front)
    F1x = F1(1);
    
    % Lateral force in the go kart frame (front)
    F1y = F1(2);
    
    frontabcorr = F1x;
    
    % Forward force (acc) (rear)
    F2x = AB;
    
    % lateral force, depending on side slip and forward force fx (AB) (rear)
    F2y1 = simpleaccy(VELY-l2*VELROTZ,VELX,(AB+TV/2)/f2n)*f2n/2;
    F2y2 = simpleaccy(VELY-l2*VELROTZ,VELX,(AB-TV/2)/f2n)*f2n/2;
    
    % lateral force of the back axle (rear)
    F2y = simpleaccy(VELY-l2*VELROTZ,VELX,AB/f2n)*f2n;
        
    % Track width of the rear axle
    w = 1;
    
    % Torque Vectoring
    TVTrq = TV*w;
    
    %accelerations in the principal directions in Go-kart frame
    % dot_dot_Phi 
    ACCROTZ = (TVTrq + F1y*l1 -F2y*l2)/Ic;%ACCROTZ = TVTrq + F1y*l1;
    
    % Acceleration in X dot_dot_x=dot_y*dot_phi+F1x+F2y+Drag?
    ACCX = F1x+F2x+VELROTZ*VELY;
    
    % Acceleration in Y dot_dot_y=-dot_x*dot_phi+F2y1+F2y2+F1y
    ACCY = F1y+F2y1+F2y2-VELROTZ*VELX;
end

