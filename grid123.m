clear
clc
%creat the map
map=[  0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0
       0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
       0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 1 0 0 0 0
       1 0 0 0 0 0 1 0 1 0 0 0 1 0 0 0 0 0 0 0
       0 1 1 1 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0
       0 1 1 1 0 0 1 1 1 0 0 0 0 0 0 1 0 0 0 0
       0 1 1 0 0 0 0 0 0 0 0 0 1 1 0 1 0 0 0 0
       0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0
       0 0 0 0 0 0 0 1 1 0 0 0 1 1 0 0 0 0 0 0
       0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0
       0 0 0 0 1 0 0 0 0 0 0 0 1 1 0 0 1 1 0 0
       0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 1 0 0
       0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0
       0 0 1 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0
       0 0 0 0 0 0 1 0 0 1 1 1 0 0 0 0 0 1 0 0
       0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 0 0 1 0 0
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
initial = [1 2];
goal = [20 20];
f = [];%heuristic values of every cell. estimated as the straight distance between the goal and cell
h = [];
g = [];
for i=1:msize(1)
    for j=1:msize(2)
        h(i,j) = sqrt((i-goal(1))^2+(j-goal(2))^2); % abs(i - goal(1)) + abs(j - goal(2));%
        g(i,j) = sqrt((i-initial(1))^2+(j-initial(2))^2); % abs(i - initial(1)) + abs(j - initial(2)); %mahaten distance
        if map(i,j) == 1
            h(i,j) = h(i,j) + 1000;
        end
        f(i,j) = h(i,j) + g(i,j);
    end
end

open=[initial(1),initial(2),f(initial(1),initial(2)),-1,-1];
%open nodes,[X(x coordinate of the cell/column),Y( y coordinate of the
%cell/row),f(sum of weight of all previous nodes and included this+heuristic), x
%(previous cell),y (previous cell)]
closed=[];%explored nodes
[m n] = min(open(:,3)); % get the mini of the h(n), m = min(h), n = index 
current = open(n,:);

%time cost
v0 = 1.0 %m/s
% T0 = dis/v0
Ta = 1.0 %s
Td = 1.0 %s
Tr = 5.0 %s

while ~isequal(current(1:2), goal) % not goal
    
    %children = expand_node(current, f, msize(1), msize(2));
    children = expand_node(current, h, msize(1), msize(2));
    
    % place the children with the lower cost in the open list if they are，redundant in that list
    closed = [closed; current]; % add current to the close list
    open = open([1:n-1, n+1:end],:); % deleting the current element from the open
    
    if isempty(children)==0 % not empty
        for i=1:length(children(:,1)) % look up the list of the children
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % caculate gn Tc2n
            childer  = children(i, :);
            child = childer(1:2);
            Tc2n = 0;
            if(current(4) ~= -1 || current(5) ~= -1 )
                father = current(4:5);
                squre = (child(1)-father(1))^2 + (child(2)-father(2))^2;
                if(squre == 2) % thita != pi
                    Tc2n = 1.414 / v0 + Ta + Td + Tr;
                else
                    Tc2n = 2/v0;
                end
            end
            
            % caculate hn Tn2g
            has_ob = 0;
            if( child(1) == goal(1) ) %goal and child in y line
                for it = (child(2) + 1) : (goal(2) - 1)
                    if map(child(1), it) == 1
                        has_ob = 1;
                        break;
                    end
                end
            elseif ( child(2) == goal(2) ) %goal and child in lie same
                for it = (child(1) + 1) : (goal(1) - 1)
                    if map(it, child(2)) == 1
                        has_ob = 1;
                        break;
                    end
                end
            else % not in line
                has_ob = 2;
                
                has_ob2 = 0;
                has_ob3 = 0;
                for it = (child(1) + 1) : (goal(1) - 1) % lie bu bian 
                    if map(it, child(2)) == 1
                        has_ob2 = 1;
                        break;
                    end
                end
                
                for it = (child(2) + 1) : (goal(2) - 1) % heng bu bian 
                    if map(child(1), it) == 1
                        has_ob3 = 1;
                        break;
                    end
                end
                
                if has_ob2==1 && has_ob3==1
                    has_ob = 3;
                else
                    has_ob = 2;
                end
                
            end
            
            distance = sqrt((child(1)-goal(1))^2+(child(2)-goal(2))^2); %abs(goal(2) - child(2)) + abs(goal(2) - child(1)) %mahaten distance
            if has_ob == 0 % no obstacles
                Tn2g = distance / v0 + Ta + Td;
            elseif has_ob == 1 % has obstaclse
                Tn2g = distance / v0 + 3 * (Ta + Td + Tr);
            elseif has_ob == 2 % not in line hase no obstacles
                Tn2g = distance / v0 + Ta + Td + Tr;
            elseif has_ob == 3 % not in same line and hase obstacles
                Tn2g = distance / v0 + 2*(Ta + Td + Tr);
            else
                % do nothing
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %if comment this, will the original AStar,else AStar improved!
            children(i, 3) = children(i, 3) + Tn2g + Tc2n; % not add the g(n), infact ,with accureate hn, can have not gn
            
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
    row = path(i, 1);
    col = path(i, 2);
    text(col - 0.5 - 0.15, 20 -row + 0.5 + 0.1, sprintf('%d',map(row,col)));
    %plot(path(i,1)+0.5,path(i,2)+0.5,'Color',0.2*ones(3,1),'LineWidth',4);
    %hold on
    %drawnow;
end
disp('will set the path on the figrure');
