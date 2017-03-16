function [ Bot ] = load_bot_parameters( )

Bot.Kt   = 0.146;       %Motor torque contant (Nm/Apeak)
Bot.Mass_Kg = 118;%109;   %Unladen bot mass (Kg)
Bot.Inertia = 28;    %Unladen bot rotational intertia about Cg (Kg*m^2)
Bot.CgX_m = 0.36;    %Unladen bot center of mass, X coordinate, meters (in bot coordinate frame)
Bot.CgY_m = 0;%-0.053;  %Unladen bot center of mass, Y coordinate, meters (in bot coordinate frame)
Bot.WheelBase_m = 0.715;%effective rear wheel base distance (wheel to wheel)
Bot.WheelRadius_m = 0.099; %Wheel radius (m)
Bot.CasterPosX_m = 0.97;  %X position of the caster wheels, meters (in bot coordinate frame)
Bot.CasterStaticMu = 0.0196;  %Caster static coefficient of friction
Bot.CasterDynamicMu = 0.011; %Caster dynamic coefficient of friction
Bot.CasterDamping = 0.8;  %Caster damping coefficient for each caster wheel (N/(m/s))
Bot.TracStaticMu = 0.016;  %Traction wheel static coefficient of friction
Bot.TracDynamicMu = 0.017; %Traction wheel dynamic coefficient of friction
Bot.TracWheelDamping = 1.2;  %Trac wheel damping coefficient for each trac wheel (N/(m/s))
Bot.Corners=[-0.154  -0.154 1.14 1.14;   %Corners of the bot body
              0.38  -0.38 0.38 -0.38];
Bot.Corners=[-0.1  -0.1 1.14 1.14;   %Corners of the bot body
              0.38  -0.38 0.38 -0.38];
Bot.LineSensorOffsets = [-.154, 1.069];
Bot.PayloadCenterX_m = 0.51825;  %Distance from the center of the rear wheels to the center of the payload bay


end

