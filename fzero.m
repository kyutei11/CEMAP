# 1-Dim nonlinear solver

function [x,itl] = fzero(f,x0,para,tol)

	if nargin<4,
		tol = 1e-6;
	end

	f0 = feval(f,x0,para);
	x1 = x0 + 0.01;
	f1 = feval(f,x1,para);
		
	for i=1:200,
		dx = x1-x0;
		x0 = x1;
		x1 = x1-dx*f1/(f1-f0);
		f0 = f1;
		f1 = feval(f,x1,para);
		
		if norm(f1) < tol,
			x = x1;
			itl=i;
			return
		end
	end

endfunction

