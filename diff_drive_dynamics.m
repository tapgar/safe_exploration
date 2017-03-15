function [ xdd, ydd, tdd ] = diff_drive_dynamics( bot, vx, vy, w, uL, uR )


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
   pF = -(pV./norm(pV)).*bot.TracDynamicMu*bot.Mass_Kg/4 - pV.*bot.TracWheelDamping; 
else
   pF = [-bot.TracStaticMu*bot.TracStaticMu*bot.Mass_Kg/4; 0];
end

mT = [1, 0;
      0, 1;
      bot.WheelBase_m/2, 0];
pW = mT*pF;

mT = [1, 0, -bot.WheelBase_m/2;
      0, 1, 0];

pV = mT*oV;

if (norm(pV) > V_STATIC)
   pF = -(pV./norm(pV)).*bot.TracDynamicMu*bot.Mass_Kg/4 - pV.*bot.TracWheelDamping; 
else
   pF = [-bot.TracStaticMu*bot.TracStaticMu*bot.Mass_Kg/4; 0];
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


Fy = bot.Mass_Kg*bot.CgX_m*(Mz + bot.CgY_m*Fx - bot.Mass_Kg*bot.CgX_m*w*vx...
    - bot.Mass_Kg*bot.CgY_m*w*vy)/(bot.Inertia + bot.Mass_Kg*bot.CgX_m^2)...
    + bot.Mass_Kg*bot.CgY_m*w*w*bot.Inertia/(bot.Inertia + bot.Mass_Kg*bot.CgX_m^2);

max_corner_force = 100;
if abs(Fy + pW(2)) > max_corner_force
    display('slipping!');
else
    
    tdd = (Mz + bot.CgY_m*Fx - bot.CgX_m*Fy - bot.Mass_Kg*bot.CgX_m*w*vx...
    - bot.Mass_Kg*bot.CgY_m*w*vy)/bot.Inertia;
    xdd = (Fx/bot.Mass_Kg) + bot.CgY_m*tdd - bot.Cgx_m*w^2;
    ydd = 0;
    
end

end

