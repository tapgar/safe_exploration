clear
close all

env = PendulumEnv();
env = env.NominalTrajectory();
u = STOMP(env, 10);