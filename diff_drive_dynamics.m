function [ xdd, ydd, tdd ] = diff_drive_dynamics( bot, vx, vy, w, uL, uR, corner_dynamic, corner_static, max_corner_force )


g = 9.806;

fL = bot.Kt*uL/bot.WheelRadius_m;
fR = bot.Kt*uR/bot.WheelRadius_m;

fX = fL+fR;
mZ = (fR-fL)*bot.WheelBase_m/2;

oW = [fX; 0; mZ];

V_STATIC = 0.01;

mT = [1, 0, bot.WheelBase_m/2;
      0, 1, 0];

oV = [vx; vy; w];

pV = mT*oV;

if (norm(pV) > V_STATIC)
   pF = -(pV./norm(pV)).*bot.TracDynamicMu*bot.Mass_Kg*g/4 - pV.*bot.TracWheelDamping; 
else
    if (norm(pV) > 0.00001)
       pF = -(pV./norm(pV)).*[bot.TracStaticMu*bot.Mass_Kg*g/4; 0];
    elseif (norm(oW) > 0.00001)
        pF = -(oW(1:2,1)./norm(oW(1:2,1))).*[bot.TracStaticMu*bot.Mass_Kg*g/4; 0];
    else
        pF = [0;0];
    end
end

mT = [1, 0;
      0, 1;
      bot.WheelBase_m/2, 0];
pW = mT*pF;

mT = [1, 0, -bot.WheelBase_m/2;
      0, 1, 0];

pV = mT*oV;

if (norm(pV) > V_STATIC)
   pF = -(pV./norm(pV)).*bot.TracDynamicMu*bot.Mass_Kg*g/4 - pV.*bot.TracWheelDamping; 
else
    if (norm(pV) > 0.00001)
       pF = -(pV./norm(pV)).*[bot.TracStaticMu*bot.Mass_Kg*g/4; 0];
    elseif (norm(oW) > 0.00001)
        pF = -(oW(1:2,1)./norm(oW(1:2,1))).*[bot.TracStaticMu*bot.Mass_Kg*g/4; 0];
    else
        pF = [0;0];
    end
end

mT = [1, 0;
      0, 1;
      -bot.WheelBase_m/2, 0];
  
pW = pW + mT*pF;

if sign(pW(1) + oW(1)) ~= sign(oW(1))
    oW(1) = 0;
    pW(1) = 0;
end
if sign(pW(3) + oW(3)) ~= sign(oW(3))
    pW(3) = 0;
    oW(3) = 0;
end

Fx = oW(1) + pW(1);
Mz = pW(3) + oW(3);

cx = bot.CgX_m;
cy = bot.CgY_m;
m = bot.Mass_Kg;
Izz = bot.Inertia;

Fy = (m*cx*(Mz + Fx*cy - m*w*(cx*vx + cy*vy)) - Izz*m*cy*w^2)/(Izz + m*cx^2);

if abs(Fy + pW(2)) > max_corner_force
    display('slipping!');
    Fy = -corner_static*m*g*Fy/abs(Fy);
    tdd = (Mz + Fx*cy - Fy*cx - m*w*(cx*vx + cy*vy))/Izz;
    ydd = Fy/m + cy*w^2 - cx*tdd;
    xdd = Fx/m + cy*tdd + cx*w^2;
    
else   
    tdd = (Mz + Fx*cy - Fy*cx - m*w*(cx*vx + cy*vy))/Izz;
    xdd = (Fx/m) + cy*tdd + cx*w^2;
    ydd = 0;
    if (abs(vy) > 0)
        ydd = -vy*4*corner_dynamic*g;
    end
end

end

