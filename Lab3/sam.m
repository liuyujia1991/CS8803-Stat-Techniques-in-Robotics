function K = sam(x1,sigma,varargin)

switch nargin
    case 2
        D = x1*x1';
        K = exp(-acos(D).^2/(2*sigma^2));
    case 3
        x2=varargin{1};
        D = x1*x2';
        K = exp(-acos(D).^2/(2*sigma^2));
end
