function d = costfit(param,SysID)

    l = 1.19;
    l1 = 0.73;
    l2 = l-l1;

    t = SysID(:,1);
    dt = (t(1001)-t(1))/1000;
    tms = SysID(:,2);
    vx = SysID(:,3);
    vy = SysID(:,4);
    vr = SysID(:,5);
    ar = getDerivation(vr, 60, dt);
    ax = SysID(:,6);
    ay = SysID(:,7);
    sax = gaussfilter(ax,100);
    say = gaussfilter(ay,100);
    scay = say+l2*ar;
    cvy = vy+l2*vr;
    s = SysID(:,8);
    beta = -0.58.*s.*s.*s+0.93*s;
    kinrot = vx/l.*tan(beta);
    pl = SysID(:,9);
    pr = SysID(:,10);
    pal = SysID(:,11);
    par = SysID(:,12);
    ptv = (par-pal)/2;
    vwx = SysID(:,13);
    [m,~]=size(SysID);
    step = 10000;
    d=0;
    for i = 1:step:m
        if(vx(i)>3)
            [ACCX,ACCY,ACCROTZ] = modelDx(vx(i),cvy(i),vr(i),beta(i),ax(i),ptv(i), param);
            d=d+(ACCY-scay(i))^2;
            d=d+(ACCROTZ-ar(i))^2;
        end
    end
end

