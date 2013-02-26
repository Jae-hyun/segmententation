classdef universe < handle
    properties
        name = 'Universe'
        num
        elts=struct;
    end % properties
    methods (Access = public)
        function obj=initialize(obj, elements)
            obj.num = elements;
%             obj.elts = zeros(elements, 3);
            for i=1:1:elements
                obj.elts(i).rank = 0;
                obj.elts(i).size = 1;
                obj.elts(i).p = i;
            end
        end
        function test=find(obj, x)
            y = x;
            while (y ~= obj.elts(y).p)
                y = obj.elts(y).p;
            end
            obj.elts(x).p = y;
            test = y;
        end
        function join(obj, x, y)
            if obj.elts(x).rank > obj.elts(y).rank
                obj.elts(y).p = x;
                obj.elts(x).size = obj.elts(x).size + obj.elts(y).size;
            else
                obj.elts(x).p = y;
                obj.elts(y).size = obj.elts(y).size + obj.elts(x).size;
                if obj.elts(x).rank == obj.elts(y).rank
                    obj.elts(y).rank = obj.elts(y).rank + 1;
                end
            end
            obj.num = obj.num - 1;
                
        end
        function test=size(obj, x)
            test = obj.elts(x).size;
        end
        function test=num_sets(obj)
            test = obj.num;
        end
    end % methods
end % classdef