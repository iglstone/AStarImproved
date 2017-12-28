clear
clc
%creat the map
map=[  0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0
       0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
       0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 1 0 0 0 0
       0 0 0 0 0 0 1 0 1 0 0 0 1 0 0 0 0 0 0 0
       0 1 1 1 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0
       0 1 1 1 0 0 1 1 1 0 0 0 0 0 0 1 0 0 0 0
       0 1 1 1 0 0 0 1 1 1 0 0 0 1 0 1 0 0 0 0
       0 1 1 0 0 0 0 0 0 0 0 0 1 1 0 1 0 0 0 0
       0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0
       0 0 0 0 0 0 0 1 1 0 0 0 1 1 0 0 0 0 0 0
       0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0
       0 0 0 0 1 0 0 0 0 0 0 0 1 1 0 0 1 1 1 0
       0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 1 1 0
       0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 0
       0 0 1 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0
       0 0 0 0 0 0 1 0 0 1 1 1 0 0 0 0 0 1 1 0
       0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 0 0 1 1 0
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; 

%msize=size(map,1); % G 地形图为01矩阵，如果为1，表示障碍物 
msize = size(map);
mlength = msize(1);

%fill the map with color
for i=1:msize(1)
   for j=1:msize(2)
       x1 = j-1;     x2 = j;   x3 = x2;            x4 = x1;
       y1=mlength-i; y2 = y1;  y3 = mlength -i +1; y4 = y3;
       if map(i,j)==1 
           fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.2,0.2,0.2]); 
           hold on
       else 
           fill([x1,x2,x3,x4],[y1,y2,y3,y4],[1,1,1]); 
           hold on
       end 
   end
end

%astar to search the path
initial = [1 1];
goal = [19 19];
h=[];%heuristic values of every cell. estimated as the straight distance between the goal and cell
for i=1:msize(1)
    for j=1:msize(2)
        h(i,j)=sqrt((i-goal(1))^2+(j-goal(2))^2);
        if map(i,j)==1
            h(i,j)=h(i,j)+1000;
        end
    end
end

open=[initial(1),initial(2),h(initial(1),initial(2)),-1,-1];
%open nodes,[X(x coordinate of the cell/column),Y( y coordinate of the
%cell/row),f(sum of weight of all previous nodes and included this+heuristic), x
%(previous cell),y (previous cell)]
closed=[];%explored nodes
[m n] = min(open(:,3)); % get the mini of the h(n), m = min(h), n = index 
current = open(n,:);

while ~isequal(current(1:2), goal) % not goal
    
    children = expand_node(current, h, msize(1), msize(2));
    
    % place the children with the lower cost in the open list if they are，redundant in that list
    closed = [closed; current]; % add current to the close list
    open = open([1:n-1, n+1:end],:); % deleting the current element from the open
    
    if isempty(children)==0 % not empty
        for i=1:length(children(:,1)) % look up the list of the children
            op_index = BinA(open(:,1:2),children(i,1:2));
            if op_index~=0 % if the children in the open list
                if(children(i,3) < open(op_index,3))% if the child has a less expensive cost, put it to the open list
                    open(op_index,:) = children(i,:);
                end
            else % if the children not in the open list
                cl_index = BinA(closed(:,1:2),children(i,1:2));
                if cl_index == 0 % if not in the close list, move to the open list
                    open = [open ; children(i,:)];
                else % if in the close list 
                    if (children(i,3) < closed(cl_index,3)) % if children has the less cost, will replace the close list with children
                        %closed(op_index,:)=children(i,:);
                        closed(cl_index,:)=children(i,:);
                    end
                end
            end
        end
    end
    [m n] = min(open(:,3));% get the min cost index
    current = open(n,:);
end

closed=[closed ;current];
%reiterate to start to find path
node=current;
path=current(1:2);
while ~isequal(node(4:5),initial)
    path=[node(4:5); path];
    node=closed(BinA(closed(:,1:2),node(4:5)),:);
end

x=[];y=[];
for i=1:length(path(:,1))
    fprintf('path x： %d, y:%d \n', path(i,1), path(i,2));
end
disp('will set the path on the figrure');
