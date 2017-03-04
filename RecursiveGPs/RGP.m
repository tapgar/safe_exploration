classdef RGP
    %Recursive Gaussian Process
    
    properties
        nX %number of basis vectors
        nD %dimensionality of inputs
        nY %number of outputs
        ss %signal stdev
        sm %output stdev
        w  %SE kernel width
        X %basis vectors
        
        K  %K(X,X)
        Kinv
        k  %K(X,x)
        
        C  %nX x nX covariance matrix
        mu %nX mean func
        
        Y %values for basis points... initially all zeros
        cY
    end
    
    methods
        
        function obj = RGP(X, ss, sm, w)
           
            obj.X = X;
            obj.ss = ss;
            obj.sm = sm;
            obj.w = w;
            obj.nD = length(obj.w);
            obj.nX = length(obj.X);
            obj.nY = length(obj.sm);
            
            obj.K = zeros(obj.nX,obj.nX);
            for n = 1:1:obj.nX
                x = ones(obj.nX,1)*obj.X(n,:) - obj.X;
                knew = obj.ss*exp(-0.5*sum(inv(obj.w)*x'.^2,1));
                obj.K(n,:) = knew;
            end
            obj.Kinv = inv(obj.K);
            obj.C = obj.K;
            
            obj.mu = zeros(obj.nX,obj.nY);
            obj.Y = zeros(obj.nX,obj.nY);
            obj.cY = 100000.*ones(obj.nX,obj.nY);
        end
        
        function yp = predict(obj, x)
            
        end
        
        function v = getBasisVar(obj)
           v = zeros(obj.nX,obj.nD);
           for i = 1:1:obj.nX
              kstar = obj.k_star(obj.X(i,:));
              J = kstar'*obj.Kinv;
              v(i,:) = obj.ss^2 - J*kstar + J*obj.C*J';
              %v(i,:) = obj.ss^2 - kstar'*inv(obj.K + obj.sm.*eye(obj.nX))*kstar;
           end
           if (min(v) < obj.ss^2)
               ofuck=true;
           end
        end
        
        function obj = calck(obj,x)
           obj.k = obj.k_star(x); 
        end
        
        function kstar = k_star(obj,x)
            x = ones(obj.nX,1)*x;
            dx = x - obj.X;
            kstar = obj.ss*exp(-0.5*sum(inv(obj.w)*dx'.^2,1))';
        end
        
        function SIG = getCovariance(obj)
            J = obj.k'*obj.Kinv;
            SIG = obj.ss^2 - J*obj.k + J*obj.C*J';
        end
        
        function obj = updateCovariance(obj)
            SIG = obj.getCovariance();
            J = obj.k'*obj.Kinv;
            G = obj.C*J'*inv(SIG + obj.sm^2.*eye(obj.nY));
            obj.C = (eye(obj.nX) - G*J)*obj.C;
            
%             obj.C = (eye(obj.nX) - J'*((J*obj.C*J' - J*obj.k + (obj.sm^2).*eye(obj.nY) + obj.ss^2)^-1)*J)*obj.C;
        end
        
        function obj = updateMean(obj, y)
            
            SIG = obj.getCovariance();
            J = obj.k'*obj.Kinv;
            G = obj.C*J'*inv(SIG + obj.sm^2.*eye(obj.nY));
            Ksig = inv(obj.K + (obj.sm^2).*eye(obj.nX));
            mxs = obj.k'*Ksig*obj.Y;
            mXs = zeros(obj.nX,1);
            for i = 1:1:obj.nX
                mXs(i,1) = obj.k_star(obj.X(i,1))'*Ksig*obj.Y;
            end
            mup = mxs + J*(obj.mu - mXs);
            obj.mu = obj.mu + G*(y - mup);
        end
        
        function obj = update(obj,x,y) 
           [m, n] = min(sqrt(sum(obj.w*(obj.X - x).^2,2)));
           if (m < obj.cY(n,1))
               obj.cY(n,1) = m;
               obj.Y(n,1) = y;
           end
           obj = obj.calck(x);
           
           obj = obj.updateMean(y);
           obj = obj.updateCovariance();
           imagesc(obj.C)
        end
        
        function [A, B] = linearize(obj, x, u)
            x_diff = obj.X - repmat([x,u],obj.nX,1);
            dfdx = -(1\obj.w)*x_diff'*(obj.k.*(obj.Kinv*obj.mu));
            A = dfdx(1:length(x),:);
            B = dfdx(length(x)+1:end,:);
        end
        
    end
    
end

