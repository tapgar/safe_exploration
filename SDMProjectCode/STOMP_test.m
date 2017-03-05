env = DIEnv(@DI_map_1);
env = env.NominalTrajectory();
env.U_NOM = abs(env.U_NOM + randn(100,1) * 0.1);
u_stomp = STOMP(env, 10);
figure()
plot(u_stomp(:,1))
hello = 1;