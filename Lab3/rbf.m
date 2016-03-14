function K = rbf(x1,sig,varargin)

switch nargin
    case 2
        n=size(x1,1);
        K=repmat(diag(x1*x1')',n,1)+repmat(diag(x1*x1'),1,n)-2*(x1*x1');
    case 3
        x2=varargin{1};
        n=size(x1,1);
        n2=size(x2,1);
        K=repmat(diag(x2*x2')',n,1)+repmat(diag(x1*x1'),1,n2)-2*(x1*x2');
    otherwise
        disp 'error, wrong number of arguments to rbf function'
end

K=exp((-1/(2*sig^2))*K);
