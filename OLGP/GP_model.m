classdef GP_model
    
    properties

        X       %model data input
        y       %model output
        c       %model center
        
        num_pts %number of points in model
        max_num_pts     % max num of points allowed
        
        hp      %model hyperparams... should be the same for all models
        
        nX      %size of a single input
        nY      %size of a single output
        
        K       %kernel and inverse
        invK
        
        X_time  %keeps track of how many iterations this sample has been in the model
        
    end
    
    methods
        
        function obj = GP_model(max_num_pts, newX, newY, hyperParams)
           
            obj.nX = length(newX);
            obj.X = zeros(max_num_pts, obj.nX);
            obj.nY = length(newY);
            obj.y = zeros(max_num_pts, obj.nY);
            
            obj.c = newX;
            obj.hp = hyperParams;
            
            obj.max_num_pts = max_num_pts;
            obj.num_pts = 0;       
            
            obj.K = zeros(max_num_pts, max_num_pts);
            obj.invK = zeros(max_num_pts, max_num_pts);
            obj.X_time = zeros(max_num_pts, 1);
            
            obj = obj.addPoint(newX,newY);
        end
        
        function obj = addPoint(obj, newX, newY)
            
            GainLimit = 0.01;
           
            if (obj.num_pts == obj.max_num_pts)
                
                %calc info gain for this sample
                infG = obj.ComputeInfoGain(newX,newY);
                if (infG > GainLimit)
                   
                    temp = [linspace(1,obj.num_pts,obj.num_pts)', obj.X_time./sum(obj.X_time)];
                    t = rand - 1e-20;
                    shuffTemp = shuffleRows(temp);
                    sig = cumsum(shuffTemp(:,2));
                    tempidx = min(find(t < sig));
                    idx = shuffTemp(tempidx,1);
                    idx = randi(obj.num_pts);
                    obj.X(idx,:) = newX;
                    obj.y(idx,:) = newY;
                    
                    obj.X_time(idx,1) = 0;
                    
                    obj = obj.ComputeKernel(newX, idx);
                    
                end
                
            else
                
                %append to list and update kernel
                obj.num_pts = obj.num_pts + 1;
                obj.X(obj.num_pts,:) = newX;
                obj.y(obj.num_pts,:) = newY;
                
                obj = obj.ComputeKernel(newX);
                
            end
            if obj.num_pts > 1
                obj.c = mean(obj.X(1:obj.num_pts,:));
            end
            obj.X_time(1:obj.num_pts,1) = obj.X_time(1:obj.num_pts,1) + ones(obj.num_pts,1);
            
        end
        
        function dist = calc_dist(obj, x)
           
            dist = exp(-0.5*((obj.c-x).^2)*diag(obj.hp.W));
            
        end
        
        function [pred, V] = get_prediction(obj, x)
           
            x = ones(obj.num_pts,1)*x - obj.X(1:obj.num_pts,:);
            Knew = obj.hp.sig_std.*exp(-0.5*(x.^2)*diag(obj.hp.W));
            
            pred = Knew'*obj.invK(1:obj.num_pts,1:obj.num_pts)*obj.y(1:obj.num_pts,:);
            V = obj.hp.sig_std - Knew'*obj.invK(1:obj.num_pts,1:obj.num_pts)*Knew;
            
        end
        
        function obj = ComputeKernel(obj, newX, rep_idx )
            
            if (nargin == 1) %recompute full kernel
                
                for n=1:1:obj.num_pts
                   
                    x = ones(obj.num_pts,1)*obj.X(n,:) - obj.X(1:obj.num_pts,:);
                    Knew = obj.hp.sig_std.*exp(-0.5*(x.^2)*diag(obj.hp.W));
                    obj.K(n,:) = Knew;
                    
                end
                obj.K = obj.K + diag(ones(obj.num_pts,1).*obj.hp.y_std);
                
            elseif (nargin == 2) %update last row only
                
                %instead of recomputing entire K=x'*W*x it will be faster
                %to just compute last row xnew.^2*W(diag)
                x = ones(obj.num_pts,1)*newX - obj.X(1:obj.num_pts,:);
                Knew = obj.hp.sig_std.*exp(-0.5*(x.^2)*diag(obj.hp.W));
                Knew(end) = Knew(end)+obj.hp.y_std;
                
                %K = [K, Knew...
                %     Knew, k];
                obj.K(obj.num_pts,1:obj.num_pts) = Knew';
                obj.K(1:obj.num_pts,obj.num_pts) = Knew;
                
                
            elseif (nargin == 3) 
                
                %instead of recomputing entire K=x'*W*x it will be faster
                %to just compute replaced row xnew.^2*W(diag)
                x = ones(obj.num_pts,1)*newX - obj.X(1:obj.num_pts,:);
                Knew = obj.hp.sig_std.*exp(-0.5*(x.^2)*diag(obj.hp.W));
                Knew(rep_idx) = Knew(rep_idx)+obj.hp.y_std;
                
                %K = [K, Knew...
                %     Knew, k];
                obj.K(rep_idx,1:obj.num_pts) = Knew';
                obj.K(1:obj.num_pts,rep_idx) = Knew;
                
            else
                error('Wrong number of inputs!!!');
            end
            
            obj.invK(1:obj.num_pts,1:obj.num_pts) = inv(obj.K(1:obj.num_pts,1:obj.num_pts));
            
        end
        
        function InfG = ComputeInfoGain(obj, newX, newY)
            
            x = ones(obj.num_pts,1)*newX - obj.X(1:obj.num_pts,:);
            Knew = obj.hp.sig_std.*exp(-0.5*(x.^2)*diag(obj.hp.W));
            
            k_star = obj.hp.sig_std + obj.hp.y_std;
            
            alpha = 1/(k_star + obj.hp.y_std - Knew'*obj.invK*Knew);
            V = [-alpha*obj.invK*Knew; alpha];
            
            temp = obj.hp.y_std*alpha;
            
            K_star = newY - (obj.hp.y_std/(temp+1))*V'*[obj.y;newY];
            InfG = 0.5*(log(1+temp) - temp/(1+temp) + (K_star.^2/obj.hp.y_std)*[Knew; k_star]'*V);
        end
        
    end
    
end

