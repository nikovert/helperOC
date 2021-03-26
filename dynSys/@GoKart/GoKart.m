classdef GoKart < DynSys
  properties
    % Thrust bounds
    wRange
    
    %Exhaust velocity
    ve
    
    % Initial mass
    m0
    
    % Acceleration dampening
    damp = 0.5
    
    % Dimensions that are active (e.g. [1 2 4])
    dims
  end
  
  methods
    function obj = GoKart(x, wRange, ve, dims)
      % obj = GoKart(initial_point)
      %     simple GoKart that tries to drive over the hill as fast as
      %     possible
      %
      % Dynamics:
      %    \dot{x}_1 = x_2
      %    \dot{x}_2 = g(x) + T/m
      %    \dot{x}_3 = -T/ve
      %    \dot{x}_4 = -x_1/x_2
      %         u \in wRange
      %
      % Inputs:
      %   x      - state: [xpos; xvel; mass; time]
      %   wRange   - thrust bounds
      %
      % Output:
      %   obj       - a GoKart object
      
      if ~iscolumn(x)
        x = x';
      end
      
      if nargin < 2
        wRange = [-1 1];
      end
      
      if nargin < 2
        ve = 0.4;
      end
      
      if nargin < 4
        dims = 1:4;
      end
      
      if numel(wRange) < 2
          wRange = [-wRange; wRange];
      end
      
      % Basic vehicle properties
      obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
      obj.nx = length(dims);
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.m0 = x(3);
      obj.ve = ve;
      obj.wRange = wRange;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef
