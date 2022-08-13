% The hw8.m file defines a 7dof kinova gen3 robot arm and an environment. 
% The environment contains 2 obstacles and a target object. 
% Implement the RRT algorithm to find a collision-free path from a start configuration
% (you can choose) to the goal configuration(predefined). Choose appropriate 
% hyperparameter values for the step size and frequency of sampling.
robot = loadrobot('kinovaGen3', 'DataFormat', 'column');
endEffector = "EndEffector_Link"; 

%%---Setting up environment---%%
base = collisionBox(1, 1, 0.01);
obstacle1 = collisionBox(0.4,1,0.02);
obstacle1.Pose = trvec2tform([0.3,0,0.6]);
obstacle2 = collisionBox(0.6,0.2,0.02);
obstacle2.Pose = trvec2tform([-0.2,0.4,0.5]);
can = collisionCylinder(0.03,0.16);
can.Pose = trvec2tform([0.3,0.0,0.7]);
env={base obstacle1 obstacle2 can};

targetPos = [-0.15,0.35,0.51]; %Goal configuration

%---Your code goes here---%%
ss = HelperRigidBodyTreeStateSpace(robot);

ss.EndEffector = endEffector;

% Define the workspace goal region (WGR)
R = [0 0 1; 1 0 0; 0 1 0];

Tw_0 = can.Pose;
Te_w = rotm2tform(R);
bounds = [0 0;
    0 0;
    0 0.01;
    0 0;
    0 0;
    -pi pi];
setWorkspaceGoalRegion(ss, Tw_0, Te_w, bounds);

disp(ss.RigidBodyTree.getBody("ForeArm_Link"));
% Customize The State Validator
sv = HelperValidatorRigidBodyTree(ss);

% Add obstacles in the environment
addFixedObstacles(sv, obstacle1, 'obstacle1', [71 161 214]/256);
addFixedObstacles(sv, obstacle2, 'obstacle2', [71 161 214]/256);
addFixedObstacles(sv, can, 'can', 'r');
addFixedObstacles(sv, base, 'base', [1, 0.5, 0]);
%%---Visualizing the environment---%%
% show(robot);
% hold on
% show(env{1})
% show(env{2})
% show(env{3})
% show(env{4})












% % Create state space and set workspace goal regions (WGRs)
% ss = ExampleHelperRigidBodyTreeStateSpace(robot);

% ss.EndEffector = 'j2s7s300_end_effector';
% 
% % Define the workspace goal region (WGR)
% R = [0 0 1; 1 0 0; 0 1 0];
% 
% Tw_0 = can.Pose;
% Te_w = rotm2tform(R);
% bounds = [0 0;
%     0 0;
%     0 0.01;
%     0 0;
%     0 0;
%     -pi pi];
% setWorkspaceGoalRegion(ss, Tw_0, Te_w, bounds);
% 
% % Customize The State Validator
% sv = exampleHelperValidatorRigidBodyTree(ss);
% 
% % Add obstacles in the environment
% addFixedObstacles(sv, obstacle1, 'obstacle1', [71 161 214]/256);
% addFixedObstacles(sv, obstacle2, 'obstacle2', [71 161 214]/256);
% addFixedObstacles(sv, can, 'can', 'r');
% addFixedObstacles(sv, base, 'base', [1, 0.5, 0]);
% 
% % Set the validation distance
% sv.ValidationDistance = 0.01;
% 
% % Set random seeds for repeatable results
% rng(0,'twister') % 0
% 
% 
% % Compute the reference goal configuration. Note this is applicable only when goal bias is larger than 0. 
% Te_0ref = Tw_0 * Te_w;
% ik = inverseKinematics('RigidBodyTree', robot);
% refGoalConfig = ik(ss.EndEffector, Te_0ref, ones(1, 6), homeConfiguration(ss.RigidBodyTree));
% 
% % Compute initial configuration
% T = Te_0ref;
% T(1, 4) = 0.3;
% T(2, 4) = 0.0;
% T(3, 4) = 0.4;
% initConfig = ik(ss.EndEffector, T, ones(1, 6), homeConfiguration(ss.RigidBodyTree));
% 
% planner = plannerRRT(ss,sv);