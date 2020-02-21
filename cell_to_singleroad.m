function [nextjunction,junction_time,last_relay,t_relay,store_wait,store_end,end_relay_junction,store_road,all_location_relay,PDR,location_vehi,num_vehi] = cell_to_singleroad(tree,endtree,waittree,locationjunction,last_relay,R,Linmap1m,roadname,junction_time,t_relay,store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part)
%此函数作用是把元胞数组中的各个路段信息按条件提取出来，方便之后的中继节点选择函数进行中继节点选择  
    global isbeacon;
    end_relay_junction = [];
    start_relay_junction = [];%初始化参数
    roadname1 = str2num(roadname(1:2));%roadname是字符串形式的路段名，例如内容为‘1012’的字符串，
    roadname2 = str2num(roadname(3:4));%这里roadname1和roadname2目的是把路段名字符串拆成构成路段的两个junction点的序号，分别存在roadname1和roadname2中
    PDR = [];
    if isempty(all_location_relay)==0
        for i = 1:length(all_location_relay(:,1))
            if all_location_relay(i,3) == last_relay(1,1) && all_location_relay(i,4) == last_relay(1,2) && (roadname1==all_location_relay(i,1) || roadname1==all_location_relay(i,2)) && (roadname2==all_location_relay(i,1) || roadname2==all_location_relay(i,2))
                start_relay_junction = all_location_relay(i,1);
                end_relay_junction = all_location_relay(i,2);
            end
        end
    else
        dis1 = (sum((locationjunction(roadname1,2:3)-last_relay).^2))^0.5/Linmap1m;%求出roadname1距last_relay的距离
        dis2 = (sum((locationjunction(roadname2,2:3)-last_relay).^2))^0.5/Linmap1m;%求出roadname2距last_relay的距离
        if  dis1 < dis2  %如果roadname1距last_relay的距离小于roadname2距last_relay的距离，则说明roadname1离last_relay更近，是last_relay所在路段的end_relay_junction（路段终点junction）
            end_relay_junction = roadname1;%end_relay_junction存储last_relay所覆盖的junction的序号，亦是该路段终点junction序号
            start_relay_junction = roadname2;%start_relay_junction存储last_relay所在路段的起始junction序号
        else
            end_relay_junction = roadname2;
            start_relay_junction = roadname1;
        end
    end
    
    if isempty(end_relay_junction)
        dis1 = (sum((locationjunction(roadname1,2:3)-last_relay).^2))^0.5/Linmap1m;%求出roadname1距last_relay的距离
        dis2 = (sum((locationjunction(roadname2,2:3)-last_relay).^2))^0.5/Linmap1m;%求出roadname2距last_relay的距离
        if  dis1 < dis2  %如果roadname1距last_relay的距离小于roadname2距last_relay的距离，则说明roadname1离last_relay更近，是last_relay所在路段的end_relay_junction（路段终点junction）
            end_relay_junction = roadname1;%end_relay_junction存储last_relay所覆盖的junction的序号，亦是该路段终点junction序号
            start_relay_junction = roadname2;%start_relay_junction存储last_relay所在路段的起始junction序号
        else
            end_relay_junction = roadname2;
            start_relay_junction = roadname1;
        end
    end
    
    store_road = [store_road;[roadname1 roadname2]];  
    if junction_time(end_relay_junction,4)==0 %如果当前last_relay所覆盖的junction未存储过时间
        junction_time(end_relay_junction,4) = t_relay;%更新时间为当前时间
        junction_time(end_relay_junction,5) = start_relay_junction;%更新时间为当前relay的来时路段
        if isempty(find(end_relay_junction == endtree(:), 1))==0%真为1，假为0，isempty(...)==0表示不为空，此语句表示如果当前last_relay所覆盖的junction是终结junction
            if end_relay_junction == 10 && junction_time(12,4) == 0 %如果12号junction打点时间是0，然后last_relay覆盖的junction序号为10号，说明这是刚开始消息广播
                store_end(end_relay_junction,1) = end_relay_junction;
                nextjunction=12;%nextjunction用来存储与被覆盖junction相连的其他junction点的序号，这里初始化存储的是12号junction点
            else
                store_end(end_relay_junction,1) = end_relay_junction;
                store_end(end_relay_junction,2:3) = last_relay;
                store_end(end_relay_junction,4) = t_relay;
                nextjunction = 0;%nextjunction存0，表示此消息支路终结
            end
        elseif isempty(find(end_relay_junction == waittree(:), 1))==0%真为1，假为0，isempty(...)==0表示不为空，此语句表示如果当前last_relay所覆盖的junction是等待junction
            %第一次覆盖等待junction，我们让它继续跑下去
            store_wait(end_relay_junction,1) = end_relay_junction;
            store_wait(end_relay_junction,2:3) = last_relay;
            store_wait(end_relay_junction,4) = t_relay;
            store_wait(end_relay_junction,5) = start_relay_junction;
            nextjunction = tree(end_relay_junction,2:5);%将tree中与当前覆盖junction相连的junction提取出来放入nextjunction数组中
            nextjunction(:,nextjunction(1,:)==start_relay_junction)=[];%将nextjunction中包含的路段起始junction序号删掉，因为我们是从这条路来的，没必要倒回去
        else    %此语句表示如果当前last_relay所覆盖的junction是普通junction
            nextjunction = tree(end_relay_junction,2:5);%将tree中与当前覆盖junction相连的junction提取出来放入nextjunction数组中
            nextjunction(:,nextjunction(1,:)==start_relay_junction)=[];%将nextjunction中包含的路段起始junction序号删掉，因为我们是从这条路来的，没必要倒回去            
        end
        

%我们默认每个等待junction（waittree(i)）如果在覆盖过程中出现了覆盖时间和last_relay的更新，那么只能是：第二次覆盖时间小于第一次覆盖时间，从而产生的更新。
%我们基本不考虑第三次覆盖或第四次覆盖对前面覆盖时间的更新，因为这种可能性太小了
    elseif junction_time(end_relay_junction,4)~=0%如果当前last_relay所覆盖的junction已有储存时间
        if junction_time(end_relay_junction,4)>t_relay%如果junction储存的时间大于t_relay，即大于当前last_relay覆盖至此的时间，说明当前last_relay是在junction打卡时间前到的，故更新打卡时间，并以此last_relay信息为主继续此支路消息传播            
            junction_time(end_relay_junction,4) = t_relay;%更新junction_time对应位置存储的时间     
            junction_time(end_relay_junction,5) = start_relay_junction;%更新时间为当前relay的来时路段
            if isempty(find(end_relay_junction == endtree(:), 1))==0%如果覆盖的是终结junction，那么更新时间和last_relay，并终结此消息支路
                store_end(end_relay_junction,1) = end_relay_junction;
                store_end(end_relay_junction,2:3) = last_relay;%更新信息
                store_end(end_relay_junction,4) = t_relay;
                nextjunction = 0;%这里表示直接终结了
            elseif isempty(find(end_relay_junction == waittree(:), 1))==0%如果覆盖的是等待junction，那么更新时间和last_relay，并以此last_relay信息为主继续此支路消息传播
                store_wait(end_relay_junction,1) = end_relay_junction;%更新信息
                store_wait(end_relay_junction,2:3) = last_relay;%更新信息
                store_wait(end_relay_junction,4) = t_relay;
                store_wait(end_relay_junction,5) = start_relay_junction;
                %直接进入反向遍历函数
                [all_location_relay,junction_time,store_road,PDR,location_vehi,num_vehi]...
                    = reverse_traversal(store_road,end_relay_junction,start_relay_junction,tree,all_location_relay,location_vehi,num_vehi,locationjunction,last_relay,R,t_relay,junction_time,endtree,N_iter,N_part);               
                nextjunction = 0; %反向遍历之后，直接给nextjunction置零，表示可以直接终结了，因为我们在反向遍历函数中已经进行了操作
            else  %此语句表示如果当前last_relay所覆盖的junction是普通junction
                nextjunction = tree(end_relay_junction,2:5);%将tree中与当前覆盖junction相连的junction提取出来放入nextjunction数组中
                nextjunction(:,nextjunction(1,:)==start_relay_junction)=[];%将nextjunction中包含的路段起始junction序号删掉，因为我们是从这条路来的，没必要倒回去
            end
        else%如果junction储存的时间小于t_relay，说明此条消息支路后到
            %那没必要再往下走了，因为已经晚了
            nextjunction = 0;%这里表示直接终结了,因为来晚了，没必要再顺着此支路往下走了
        end
    end
    nextjunction(:,nextjunction(1,:)==0)=[];%将nextjunction中的零元素去掉
    if isempty(nextjunction)%如果nextjunction为空，即覆盖了终结节点、等待节点或者打点时间在已有时间的后面，那么我们中止此方向上的消息传播
        t_relay=[];
        end_relay_junction=[];
        last_relay = [];
    elseif length(nextjunction)<3%但凡nextjunction不为空，我们就将nextjunction补零元素到3列，以求主函数中nextjunction数组集合的结构工整
        for j = 1:3-length(nextjunction)
            nextjunction = [nextjunction,0];
        end
    end
    %有了nextjunction信息，我想做个同层循环遍历，就是循环跑完同一树层的junction点，然后再开始下一层的junction遍历，以此类推，
    %所以我们需要用几个数组来存储各种信息，这样可以实现本层每个消息分支终止点的信息待本层遍历结束后，可用作下一层的每个对应消息分支的起始点信息使用。
    
    


