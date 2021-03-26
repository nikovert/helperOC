%% Setup path
addpath(genpath('../Add-Ons/level-set-methods-toolbox/Kernel/'));

%% Should we compute the trajectory?
compTraj = true;
dims = [1, 2, 3, 4];
projDim = [1 1 0 0];

%% Grid
grid_min = [0.0; -3.5; 0.1; -0.1]; % Lower corner of computation domain
grid_max = [2.5;  3.5; 1.3; 1.3];    % Upper corner of computation domain
N = [41; 41; 21; 21];         % Number of grid points per dimension
g = createGrid(grid_min(dims), grid_max(dims), N(dims));

% boundary for the mask
min_bnd = [-1; -3; 0.5; 0.0];
max_bnd = [ 4;  3; 1.5; 1.5];

%% target set
R = 0.1;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
data0 = shapeSphere(g, [0; 0; 1; 1], R);

%% time vector
t0 = 0;
tMax = 4;
dt = 0.01;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
ve = 100;
wMax = 5;

% control trying to min or max value function?
uMode = 'min';
dMode = 'max';

x_0 = [2; 0; 1; 1];

%% Pack problem parameters

% Define dynamic system
dCar = GoKart(x_0, wMax, ve, dims);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'veryHigh'; %set accuracy
schemeData.uMode = uMode;
schemeData.dIn = [0; 0; 0];

HJIextraArgs.stopInit = x_0(dims);
mask = shapeRectangleByCorners(g, min_bnd(dims), max_bnd(dims));
HJIextraArgs.obstacleFunction = -mask;

%HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
%HJIextraArgs.visualize.valueFunction = 1;
%HJIextraArgs.visualize.initialValueFunction = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update

% uncomment if you want to see a 2D slice
HJIextraArgs.visualize.plotData.plotDims = projDim; %plot x, y
HJIextraArgs.visualize.plotData.projpt = ones(g.dim-2,1); %project at ...

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'none', HJIextraArgs);

%% Compute optimal trajectory from some initial state
if compTraj
  
  %set the initial state
  xinit = x_0(dims);
  
  figure(6)
  clf
  switch length(dims)
    case 2
        data_f = data(:,:,end);
        s = scatter(xinit(1), xinit(2));
    case 3
        data_f = data(:,:,:,end);
        s = scatter3(xinit(1), xinit(2), xinit(3));
    case 4
        data_f = data(:,:,:,:,end);
        s = scatter3(xinit(1), xinit(2), xinit(3));
  end
  s.SizeData = 70;
  hold on
  h = visSetIm(g, data_f);
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data_f,xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    
    dCar.x = xinit; %set initial state of the system
    dCar.xhist = xinit;
    
    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.dMode = dMode;
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 2; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = projDim; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,g.dim+1);
    
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);
    
    %% Plot trajectory
    [a, v] = dCar.acceleration(0:0.01:xinit(1));
    figure(4)
    for i=1:length(traj)
        hold off
        plot(0:0.01:xinit(1), v)
        [~, h] = dCar.acceleration(traj(1,i));
        hold on
        plot(traj(1,i), h, 'ko')
        drawnow
        pause(0.1)
    end
  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end