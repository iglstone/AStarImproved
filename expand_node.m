function [children]=expand_node (parent, h, V, H)
children=[];
for i=-1:1
    for j=-1:1
        
        if(i*j == 1 || i*j == -1)
            continue;
        end
        
        if j~=0 && i~=0 && parent(1)+i>0 && parent(1)+i<=V && parent(2)+j>0 && parent(2)+j<=H
            disp('here will not arrived! else wrong');
            c_child=parent(3)+1.4142+h(parent(1)+i,parent(2)+j)-h(parent(1),parent(2));
            if ((parent(1)+i)~=parent(4)|| (parent(2)+j)~=parent(5))% && (((j==0 || i==0) && j~=i)||(j~=0 && i~=0))
                children=[children; [parent(1)+i, parent(2)+j, c_child, parent(1),parent(2)]];
            end
        elseif (j==0 || i==0) && j~=i && parent(1)+i>0 && parent(1)+i<=V && parent(2)+j>0 && parent(2)+j<=H
            
            c_parent = h(parent(1), parent(2));
            c_child = h(parent(1)+i,parent(2)+j);
            
            if(c_parent < c_child || c_child > 1000) % if the child has a bigger cost ,will continue
                continue;
            end
            
%             if(c_child < 1000)
%                 if(c_parent < c_child) % if the child has a bigger cost ,will continue
%                     continue;
%                 end
%             end
            
            children = [children; [parent(1)+i, parent(2)+j, c_child, parent(1),parent(2)]];
            
            %c = parent(3)+ 1 + h(parent(1)+i,parent(2)+j) - h(parent(1),parent(2));
            %c = 1 + h(parent(1)+i,parent(2)+j);
            %fprintf('x :%d y : %d cost : %d \n',parent(1) + i, parent(2) + j, c);
            %if ((parent(1)+i)~=parent(4)|| (parent(2)+j)~=parent(5)) %&& (((j==0 || i==0) && j~=i)||(j~=0 && i~=0))
            %    children=[children; [parent(1)+i, parent(2)+j, c, parent(1),parent(2)]];
            %end
        else
            % i = j = 0
        end
    end
end

if isempty(children)==0 % not empty
    fprintf('children lenght: %d \n', length(children(:,1)));
else
    disp('children is empty! something error!');
end
end