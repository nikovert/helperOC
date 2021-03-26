function dx = dynamics(obj, ~, x, u, d)
% Dynamics of GoKart:
%    \dot{x}_1 = x_2
%    \dot{x}_2 = g(x) + T/m
%    \dot{x}_3 = -T/ve
%    \dot{x}_4 = -x_1/x_2
%         u \in wRange
% Nikolaus Vertovec, 2021-03-25

if nargin < 5
  d = [0; 0; 0];
end

if iscell(x)
  dx = cell(obj.nx, 1);
  
  for i = 1:obj.nx
    dx{i} = dynamics_cell_helper(obj, x, u, d, obj.dims, obj.dims(i));
  end
else
  dx = zeros(obj.nx, 1);
  
  if any(ismember(obj.dims, 3))
    m = x(3);
  else
    m = obj.m0;
  end
  
  for i = 1:obj.nx
    switch obj.dims(i)
      case 1
        dx(i) = x(2);
      case 2
        dx(i) = -(obj.acceleration(x(1)) + u./m);
      case 3
        dx(i) = -abs(u)./obj.ve;
      case 4
        dx(i) = -x(1)./50;
      otherwise
        error('Only dimension 1-4 are defined for dynamics of GoKart!')
    end
  end
end
end

function dx = dynamics_cell_helper(obj, x, u, ~, dims, dim)
if any(ismember(dims, 3))
    m = x{3};
else
    m = obj.m0;
end
switch dim
  case 1
    dx = x{2};
  case 2
    dx = -(obj.acceleration(x{1}) + u./m);
  case 3
    dx = -abs(u)./obj.ve;
  case 4
    dx = -x{1}/50;
  otherwise
    error('Only dimension 1-4 are defined for dynamics of GoKart!')
end
end