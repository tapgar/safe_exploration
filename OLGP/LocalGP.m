classdef LocalGP
        
    properties
        num_models      % number of models currently instantiated
        max_num_models  % max number of models we can create
        max_num_model_pts %max pts per model
        
        model_list      %list of models
        
        hp              %hyperparams
        w_lim           %distance limit for adding a new point
    end
    
    methods
    
        function obj = LocalGP(max_num_models, max_num_pts, lim, hyperParams)
            
            obj.max_num_models = max_num_models;
            obj.max_num_model_pts = max_num_pts;      
            obj.num_models = 0;
            obj.model_list = cell(obj.max_num_models,1);
            obj.hp = hyperParams;
            obj.w_lim = lim;
            
        end
        
        function obj = add_training_data(obj,X,y)
            
            w_max = 0;
            for i=1:1:obj.num_models
               w = obj.model_list{i}.calc_dist(X);
               if (w > w_max)
                   w_max = w;
                   closest_model = i;
               end
            end
            
            if (w_max > obj.w_lim) %found close enough model
                obj.model_list{closest_model} = obj.model_list{closest_model}.addPoint(X,y);
            else
                if (obj.num_models == obj.max_num_models)
                    error('Exceeding maximimum allocated models!');
                end
                obj = obj.create_model(X,y);
            end

        end
        
        function obj = create_model(obj, X, y)
            obj.num_models = obj.num_models + 1;
            obj.model_list{obj.num_models} = GP_model(obj.max_num_model_pts, X, y, obj.hp); 
        end
        
        function [y v] = query_data_point(obj, X)
            
            w = zeros(obj.num_models,1);
            ybar = zeros(obj.num_models,obj.model_list{1}.nY);
            for i = 1:1:obj.num_models
               
                w(i) = obj.model_list{i}.calc_dist(X);
                [ybar(i,:), V(i,:)] = obj.model_list{i}.get_prediction(X);
                
            end
            w = w.^4;
            y = sum((w.*ybar))./sum(w);
            v = min(V);
%             [m i] = max(w);
%             y = ybar(i,:);
            
        end
        
    
    end
    
end

