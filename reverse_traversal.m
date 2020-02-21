function [all_location_relay,junction_time,store_road,PDR,location_vehi,num_vehi] = reverse_traversal(store_road,end_relay_junction,start_relay_junction,tree,all_location_relay,location_vehi,num_vehi,locationjunction,last_relay,R,t_relay,junction_time,endtree,N_iter,N_part)
%这是出现了重节点时间更新这一种情况的反向遍历函数，反向遍历函数分为回溯操作和前进操作
    global branchset A Linmap1m T ima2 isbeacon
    %接下来找出重节点时间更新之前的父亲节点，存入nextjunction数组中，作为回溯分支进行回溯操作,nextjunction里面的Junction就是我们消息广播的目标，而last_relay就是我们消息广播的起点
    %store_road中只更新回溯操作的路段
    nextjunction=[];%初始化nextjunction
    PDR_set = [];
    while(1)
        temp = [];%建立一个temp数组目的是为了临时存放每个junction开始至其下面分支子节点路段选出的relay信息，temp通过relayinf数组来进行更新，relayinf第1、2列存relay坐标，第3、4列存路段起点终点Junction，第5列存打卡时间
        for j = 1:length(end_relay_junction)
            nextjunction_part = tree(end_relay_junction(j),2:5);
            nextjunction_part(:,nextjunction_part(1,:)==start_relay_junction(j))=[];%清除掉nextjunction中包含的start_relay_junction（新父亲节点）
            nextjunction = [nextjunction;nextjunction_part];
            for n = 1:length(all_location_relay(:,1))%通过此代码找出老父亲junction点
                if all_location_relay(n,2) == end_relay_junction(j)%找出原来目的Junction等于现在起点junction的行
                    old_father = all_location_relay(n,1);%该行第一列存储的junction就是我们的老父亲junction，将其存入old_father中
                    break;
                end
            end
        end%更新end_relay_junction的同时要更新start_relay_junction
        
        for j = 1:length(nextjunction(:,1))
            for i = 1:length(nextjunction(j,:))
                if nextjunction(j,i)==0%如果在nextjunction的第j行第i列碰到了0元素，说明nextjunction的第j行所有元素都已完成后面的操作，可以跳出nextjunction第j行元素的循环了
                    break;
                end
                [locationmark] = findroad (nextjunction(j,i),end_relay_junction(j),branchset,last_relay(j,1:2),locationjunction);
                if isbeacon == 1                    
                    [t,location_relay,relay,location_vehi,num_vehi,PDR_part]=jyc_beacon_relay_selection(location_vehi(1:num_vehi,1:4),locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay(j));
                else
                    [t,location_relay,relay,PDR_part,location_vehi,num_vehi]=jyc_bar_complete_relay_selection(location_vehi(1:num_vehi,1:4),locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay(j));%考虑障碍物的中继节点选择
                end
                location_relay(:,3:4)=location_relay(:,2:3);%第3、4列存relay的坐标
                location_relay(:,1) = end_relay_junction(j);%第1列存路段起点junction
                location_relay(:,2) = nextjunction(j,i);%第2列存路段终点junction
                location_relay(location_relay(:,3)==0,:)=[];%清除掉location_relay中的全零行
                PDR_set = [PDR_set;PDR_part];
                
                %以上调整location_relay数组完毕               

                if junction_time(nextjunction(j,i),4)~=0 && isempty(find(nextjunction(j,i) == endtree(:), 1)) && nextjunction(j,i)~=old_father%如果是前进操作
                    %如果本次仿真的路段终点在junction_time中的打点时间不为0，且不属于终止节点，且不是老父亲节点
                    junction_time(nextjunction(j,i),4) = t;%更新junction_time中的打点时间
                    for n = 1:length(all_location_relay(:,1))
                        if all_location_relay(n,1)==end_relay_junction(j) && all_location_relay(n,2)==nextjunction(j,i)%从all_location_relay数组中找出目的等于我们的目的，终点等于我们的终点的行
                            all_location_relay(n,:)=0; %找到后，将该行全部置零
                        end
                    end
                    all_location_relay(all_location_relay(:,1)==0,:)=[];%清除掉all_location_relay中的全零行
                    all_location_relay = [all_location_relay;location_relay];%将location_relay数组内容更新加进all_location_relay数组中                    
                elseif nextjunction(j,i)==old_father %如果是回溯操作
                    %直接就更新all_location_relay数组中存储的relay节点信息                 
                    for n = 1:length(all_location_relay(:,1))
                        if all_location_relay(n,2) == end_relay_junction(j) && nextjunction(j,i)==all_location_relay(n,1)%找出原来目的Junction等于现在起点junction的，且现在目的junction等于原来起点junction的行
                            all_location_relay(n,:)=0;%将原来路段中的relay点全都置零
                        end
                    end
                    all_location_relay(all_location_relay(:,1)==0,:)=[];%清除掉all_location_relay中的全零行
                    all_location_relay = [all_location_relay;location_relay];%将location_relay数组内容更新加进all_location_relay数组中
                    for n = 1:length(store_road(:,1))
                        if store_road(n,1) == nextjunction(j,i) && store_road(n,2) == end_relay_junction(j)%从store_road中找到一个起点等于我们终点（老父亲Junction点），终点等于我们的起点
                            store_road(n,1) = end_relay_junction(j);%将其起点改为我们的起点，注意：end_relay_junction(j)是起点，nextjunction(j,i)是终点
                            store_road(n,2) = nextjunction(j,i);%将其终点改为我们的终点
                        end
                    end
                    if junction_time(nextjunction(j,i),4)>t%如果回溯过程中老父亲junction点的打点时间晚于现在的打点t时间
                        junction_time(nextjunction(j,i),4)=t;%更新打点时间
                    else%如果回溯过程中老父亲junction点的打点时间早于现在的打点t时间
                        relay = [0 0];%置零掉回溯路段的relay信息，使其无法再加入下一次回溯遍历                       
                    end
                elseif junction_time(nextjunction(j,i),4)==0%如果junction_time中的时间未打卡
                    relay = [0 0];%置零掉relay信息,使其无法加入下一次遍历
                elseif isempty(find(nextjunction(j,i) == endtree(:), 1))==0%如果碰到终止点
                    junction_time(nextjunction(j,i),4) = t;%更新junction_time中的打点时间
                    for n = 1:length(all_location_relay(:,1))
                        if all_location_relay(n,1)==end_relay_junction(j) && all_location_relay(n,2)==nextjunction(j,i)%从all_location_relay数组中找出目的等于我们的目的，终点等于我们的终点的行
                            all_location_relay(n,:)=0; %找到后，将该行全部置零
                        end
                    end
                    all_location_relay(all_location_relay(:,1)==0,:)=[];%清除掉all_location_relay中的全零行
                    all_location_relay = [all_location_relay;location_relay];%将location_relay数组内容更新加进all_location_relay数组中                    
                    relay = [0 0];%置零掉relay信息,使其无法加入下一次遍历                   
                end
                %接下来应该编写前进操作中满足了碰到终止节点或覆盖junction点打卡时间为0这两个情况中的一个，应该怎么清除掉此条消息传播方向
                %下面的代码还不清楚要不要
                relayinf(1,1:2) = relay;
                relayinf(1,3) = end_relay_junction(j);%end_relay_junction(j)是路段起点
                relayinf(1,4) = nextjunction(j,i);%nextjunction(j,i)是路段终点
                relayinf(1,5) = t;
                temp = [temp;relayinf];
            end
        end
        start_relay_junction = [];
        end_relay_junction = [];
        nextjunction = [];
        t_relay = [];
        last_relay = [];
        for j = 1:length(temp(:,1))
            if temp(j,1)~=0
                last_relay = [last_relay;temp(j,1:2)];
                start_relay_junction = [start_relay_junction,temp(j,3)];
                end_relay_junction = [end_relay_junction,temp(j,4)];
                t_relay = [t_relay,temp(j,5)];
            end
        end
        
        if isempty(last_relay)%跳出条件，如果消息传播方向被全部清除，那么就可以跳出while(1)循环
            break;
        end
    
    end
    PDR = mean(PDR_set(:,1));%计算PDR

