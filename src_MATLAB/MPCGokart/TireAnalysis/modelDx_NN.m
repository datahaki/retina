function [ACCX,ACCY,ACCROTZ] = modelDx(VELX,VELY,VELROTZ,BETA,AB,TV, param)
    %param = [B1,C1,D1,B2,C2,D2,Ic];
    B1 = param(1);
    C1 = param(2);
    D1 = param(3);
    B2 = param(4);
    C2 = param(5);
    D2 = param(6);
    Ic = param(7);
    %maxA = param(8);
    magic = @(s,B,C,D)D.*sin(C.*atan(B.*s));
    reg = 0.5;
    capfactor = @(taccx)(1-satfun((taccx/D2)^2))^(1/2);
    simpleslip = @(VELY,VELX,taccx)-(1/capfactor(taccx))*VELY/(VELX+reg);
    %simpleslip = @(VELY,VELX,taccx)-VELY/(VELX+reg);
    simplediraccy = @(VELY,VELX,taccx)magic(simpleslip(VELY,VELX,taccx),B2,C2,D2);
    simpleaccy = @(VELY,VELX,taccx)capfactor(taccx)*simplediraccy(VELY,VELX,taccx);
    %acclim = @(VELY,VELX, taccx)(VELX^2+VELY^2)*taccx^2-VELX^2*maxA^2;
    simplefaccy = @(VELY,VELX)magic(-VELY/(VELX+reg),B1,C1,D1);
    %simpleaccy = @(VELY,VELX,taccx)magic(-VELY/(VELX+reg),B2,C2,D2);



    l = 1.19;
    l1 = 0.73;
    l2 = l-l1;
    f1n = l2/l;
    f2n = l1/l;
    w = 1;
    rotmat = @(beta)[cos(beta),sin(beta);-sin(beta),cos(beta)];
    vel1 = rotmat(BETA)*[VELX;VELY+l1*VELROTZ];
    f1y = simplefaccy(vel1(2),vel1(1));
    F1 = rotmat(-BETA)*[0;f1y]*f1n;
    F1x = F1(1);
    F1y = F1(2);
    frontabcorr = F1x;
    F2x = AB;
    F2y1 = simpleaccy(VELY-l2*VELROTZ,VELX,(AB+TV/2)/f2n)*f2n/2;
    F2y2 = simpleaccy(VELY-l2*VELROTZ,VELX,(AB-TV/2)/f2n)*f2n/2;
    F2y = simpleaccy(VELY-l2*VELROTZ,VELX,AB/f2n)*f2n;
    TVTrq = TV*w;
    
    
    ACCROTZ_NOM = (TVTrq + F1y*l1 -F2y*l2)/Ic;
    %ACCROTZ = TVTrq + F1y*l1;
    ACCX_NOM = F1x+F2x+VELROTZ*VELY;
    ACCY_NOM = F1y+F2y1+F2y2-VELROTZ*VELX;
    
    w1 = [-0.14495431,-0.14495432,-0.14495602,-0.14495446,0.41547364,-0.14495444,0.35395497,-0.14495426,-0.14495546,-0.14494684,0.34043065,-0.14495444,-0.14495446,-0.14495444,-0.1449543,-0.1449549,-0.14495414,-0.14495447,-0.14495455,-0.1449546,-0.37942708,-0.14495449,-0.14495443,-0.14495449,-0.15375295,-0.14495541,-0.14495444,-0.14495231,-0.14495386,-0.14495446,-0.14495459,-0.14495447;0.04797187,0.04797184,0.04797232,0.04797188,-0.0933472,0.04797188,0.09026714,0.04797184,0.04797218,0.04796979,0.3228631,0.04797187,0.04797188,0.04797187,0.04797185,0.047972,0.04797181,0.04797189,0.04797189,0.0479719,-2.395429,0.0479719,0.04797188,0.04797188,0.05045803,0.04797217,0.04797187,0.04797131,0.04797174,0.04797188,0.04797191,0.04797188;0.02068955,0.02068952,0.02069015,0.02068957,-0.46797708,0.02068956,0.37505826,0.02068952,0.02068994,0.02068692,-0.43124238,0.02068958,0.02068959,0.02068955,0.02068953,0.02068976,0.02068947,0.02068958,0.0206896,0.02068962,0.4781551,0.02068958,0.02068956,0.0206896,0.02376826,0.02068993,0.02068956,0.02068883,0.02068938,0.02068959,0.0206896,0.0206896;-0.02295808,-0.02295806,-0.02295812,-0.02295807,-0.01532856,-0.02295806,0.09440116,-0.02295807,-0.02295811,-0.02295787,0.09273423,-0.02295808,-0.02295807,-0.02295806,-0.02295807,-0.02295809,-0.02295808,-0.02295808,-0.02295807,-0.02295808,0.20375971,-0.02295807,-0.02295808,-0.02295808,-0.02317921,-0.02295811,-0.02295807,-0.02295801,-0.02295807,-0.02295808,-0.02295808,-0.02295808;-0.01552237,-0.01552237,-0.01552261,-0.0155224,0.02353069,-0.0155224,-0.01349072,-0.01552237,-0.01552254,-0.01552134,-0.00591753,-0.0155224,-0.0155224,-0.01552239,-0.01552237,-0.01552246,-0.01552235,-0.0155224,-0.0155224,-0.01552241,-0.04917638,-0.0155224,-0.01552238,-0.0155224,-0.01677618,-0.01552252,-0.01552239,-0.01552211,-0.01552232,-0.01552239,-0.01552242,-0.0155224;-0.07911365,-0.07911357,-0.07911482,-0.07911376,-0.769549,-0.07911374,0.8018475,-0.07911362,-0.07911453,-0.07910819,0.18979104,-0.07911378,-0.07911374,-0.07911378,-0.0791136,-0.07911403,-0.07911349,-0.07911381,-0.07911382,-0.07911389,0.57913077,-0.07911368,-0.07911367,-0.07911377,-0.08544,-0.07911444,-0.07911371,-0.07911213,-0.0791133,-0.07911373,-0.07911389,-0.07911377];
    w2 = [0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624729 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849155 0.00559288 0.08507525; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559289 0.00559289 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624726 0.00559288 0.00559289 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559289 0.00559289 0.00559288 0.23849145 0.00559288 0.08507527; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559289 0.00559289 0.00559289 0.00559288 0.00559288 0.00559288 0.00559288 0.00559289 0.00559288 0.13624766 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559302 0.00559288 0.00559289 0.00559288 0.23849414 0.00559288 0.08507482; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624737 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849168 0.00559288 0.0850752 ; -0.01771216 -0.01771215 -0.01771216 -0.01771216 -0.01771215 -0.01771215 -0.01771216 -0.01771215 -0.01771214 -0.01771215 -0.01771215 -0.01771215 -0.01771215 -0.01771215 -0.01771217 -0.01771215 -0.01771215 -0.01771216  0.845456   -0.01771215 -0.01771215 -0.01771215 -0.01771215 -0.01771214 -0.01771215 -0.0177109  -0.01771216 -0.01771217 -0.01771216 -0.0628012 -0.01771216 -0.47006792; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.1362474  0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.2384917 0.00559288 0.08507519;  0.00361009  0.00361007  0.00361009  0.00361009  0.00361007  0.00361007  0.00361008  0.00361008  0.00361006  0.00361008  0.00361008  0.00361007  0.00361008  0.00361008  0.00361009  0.00361008  0.00361007  0.00361008 -0.62169075  0.00361007  0.00361008  0.00361007  0.00361007  0.00361007  0.00361008  0.00360847  0.00361008  0.0036101   0.00361008 -0.03036336  0.00361008  0.6747164 ; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624729 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849145 0.00559288 0.08507524; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624762 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849331 0.00559288 0.08507494;...
     0.00559288 0.00559288 0.00559288 0.00559287 0.00559287 0.00559287 0.00559288 0.00559287 0.00559288 0.00559288 0.00559287 0.00559288 0.00559288 0.00559288 0.00559288 0.00559287 0.00559287 0.00559287 0.13624538 0.00559287 0.00559287 0.00559287 0.00559287 0.00559287 0.00559288 0.00559301 0.00559288 0.00559288 0.00559287 0.23847988 0.00559287 0.08507694;  0.02106377  0.02106376  0.02106377  0.02106376  0.02106377  0.02106376  0.02106376  0.02106376  0.02106377  0.02106377  0.02106376  0.02106377  0.02106377  0.02106377  0.02106377  0.02106377  0.02106376  0.02106376  0.19057748  0.02106376  0.02106377  0.02106376  0.02106376  0.02106375  0.02106377  0.02106405  0.02106377  0.02106377  0.02106376 -0.46095377  0.02106377  0.29353777; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624737 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.2384917 0.00559288 0.08507517; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624732 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849173 0.00559288 0.08507519; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.1362474  0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849167 0.00559288 0.08507519; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624728 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849146 0.00559288 0.08507525; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624738 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559289 0.00559288 0.23849241 0.00559288 0.0850751 ; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624728 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849125 0.00559288 0.08507528; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.1362474  0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849176 0.00559288 0.08507517;...
     0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.1362474  0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849179 0.00559288 0.08507515; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.1362474  0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849194 0.00559288 0.08507515; -0.03063164 -0.03063164 -0.03063164 -0.03063163 -0.03063164 -0.03063162 -0.03063163 -0.03063163 -0.03063164 -0.03063163 -0.03063164 -0.03063164 -0.03063164 -0.03063163 -0.03063164 -0.03063164 -0.03063164 -0.03063163 -0.03908884 -0.03063163 -0.03063164 -0.03063163 -0.03063163 -0.03063163 -0.03063164 -0.03063201 -0.03063164 -0.03063164 -0.03063163 -2.2061975 -0.03063164 -0.04975153; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.1362473  0.00559288 0.00559289 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559289 0.00559288 0.23849168 0.00559288 0.08507526; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.1362473  0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.2384916 0.00559288 0.08507524; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624734 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849177 0.00559288 0.08507517; 0.00560237 0.00560236 0.00560237 0.00560236 0.00560237 0.00560236 0.00560237 0.00560237 0.00560237 0.00560237 0.00560237 0.00560237 0.00560237 0.00560237 0.00560237 0.00560237 0.00560236 0.00560236 0.13835342 0.00560236 0.00560237 0.00560236 0.00560236 0.00560236 0.00560237 0.0056025  0.00560237 0.00560237 0.00560237 0.2520932 0.00560236 0.08305323; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559289 0.00559288 0.00559288 0.00559288 0.13624753 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.2384932 0.00559288 0.08507495; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624734 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849162 0.00559288 0.0850752 ;...
     0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624683 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23848835 0.00559288 0.08507572; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624719 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849076 0.00559288 0.08507536; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624732 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849171 0.00559288 0.08507519; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624743 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849188 0.00559288 0.08507513; 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.13624732 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559288 0.00559301 0.00559288 0.00559288 0.00559288 0.23849174 0.00559288 0.08507516];
    w3 = [-0.00193566 -0.0560098  -0.07249359; -0.00193566 -0.0560098  -0.07249355; -0.00193566 -0.0560098  -0.07249359; -0.00193566 -0.05600976 -0.07249358; -0.00193566 -0.05600982 -0.07249358; -0.00193566 -0.05600978 -0.07249353; -0.00193566 -0.05600979 -0.07249356; -0.00193566 -0.05600979 -0.07249357; -0.00193566 -0.05600982 -0.07249355; -0.00193565 -0.05600981 -0.07249357; -0.00193566 -0.05600981 -0.07249356; -0.00193566 -0.05600981 -0.07249356; -0.00193566 -0.05600981 -0.07249356; -0.00193566 -0.05600981 -0.07249356; -0.00193566 -0.0560098  -0.07249361; -0.00193566 -0.0560098  -0.07249356; -0.00193566 -0.05600981 -0.07249355; -0.00193566 -0.05600978 -0.07249355;  0.00319804 -1.025137    0.6333159 ; -0.00193566 -0.0560098  -0.07249356; -0.00193566 -0.05600981 -0.07249356; -0.00193566 -0.05600977 -0.07249352; -0.00193566 -0.05600978 -0.07249354; -0.00193566 -0.05600979 -0.07249353; -0.00193566 -0.0560098  -0.07249357; -0.00193588 -0.05601313 -0.07249192; -0.00193566 -0.05600981 -0.07249356; -0.00193566 -0.05600979 -0.07249362; -0.00193565 -0.05600979 -0.07249357; -0.00478804  2.059327    0.7908462 ; -0.00193566 -0.05600978 -0.07249358;  0.0116015   0.1876672  -0.90573245];
    b1 = [0.05217487 0.05217485 0.05217548 0.05217491 0.2730629  0.05217491 0.21831347 0.05217485 0.05217526 0.05217226 0.04464492 0.05217493 0.05217491 0.05217491 0.05217487 0.05217505 0.05217481 0.05217495 0.05217494 0.05217497 0.7995569  0.05217491 0.05217489 0.05217491 0.05529689 0.05217526 0.0521749  0.05217415 0.05217471 0.05217491 0.05217497 0.05217492];
    b2 = [0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.14871182 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523537 0.00523544 0.00523537 0.00523537 0.00523537 0.1818405 0.00523537 0.14218244];
    b3 = [ 0.01077729 -0.0687052  -0.08270673];
    means = [2.860131,0.000018,0.000083,0.000042, 0.081740,0.000069];
    stds = [1.867398,0.338037,0.557183, 0.198449,0.829976,0.666156];

    input = [VELX,VELY,VELROTZ,BETA,AB,TV];

    normed_input = (input - means) ./ stds;

    h1 = log(exp(normed_input * w1 + b1) + 1);
    h2 = log(exp(h1 * w2 + b2) + 1);
    disturbance = h2 * w3 + b3;
    
    ACCX = ACCX_NOM + disturbance(1);
    ACCY = ACCY_NOM + disturbance(2);
    ACCROTZ = ACCROTZ_NOM + disturbance(3);
end

