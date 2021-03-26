function uOpt = optCtrl(obj, ~, x, deriv, uMode)
% uOpt = optCtrl(obj, t, x, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% Optimal control
if strcmp(uMode, 'max')
  if ismember(obj.dims, 3)
    uOpt = (deriv{2}./x{2} < deriv{3}./obj.ve)*(obj.wRange(2)) + (deriv{2}./x{2} > deriv{3}./obj.ve)*(obj.wRange(1));
  else
    uOpt = (deriv{2} < 0)*(obj.wRange(2)) + (deriv{2} > 0)*(obj.wRange(1));
  end
elseif strcmp(uMode, 'min')
  if ismember(obj.dims, 3)
    uOpt = (deriv{2}./x{2} < deriv{3}./obj.ve)*(obj.wRange(1)) + (deriv{2}./x{2} > deriv{3}./obj.ve)*(obj.wRange(2));
  else
    uOpt = (deriv{2} < 0)*(obj.wRange(1)) + (deriv{2} > 0)*(obj.wRange(2));
  end
else
  error('Unknown uMode!')
end

end