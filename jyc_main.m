function [time,PDR,location_vehi,num_vehi,maxhop]=jyc_main(R,N_iter,N_part,location_vehi,num_vehi)
%在cell_to_singleroad里面的reverse_traversal函数里也调用了jyc_bar_complete_relay_selection函数和findroad函数
%关于PDR，我们默认通信范围内至少有一个车辆节点可供通信，那么如果是由于通信范围内没有车辆节点，进而选择生成一个P_opt点与sender点之间的车辆节点作为new_relay，是不算PDR=0（也就是通信失败）的情况
%PDR=0的情况只有在：1.争用次数超过最大次数后，争用延迟无限大  or  2.基于信标的方法，选中的relay车辆行驶出通信范围  这两种情况才会出现
    global branchset A Linmap1m T ima2 locationjunction isbeacon;    
   
%     ima=imread('system_map.jpg');
%     imshow(ima);                  %显示图片
%     hold on;
% 
%     for i = 1:length(location_vehi(:,1))
%         plot(location_vehi(i,2),location_vehi(i,3),'o','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor',[0.7,0.7,0.7],'MarkerSize',4);%根据X_vehi和Y_vehi将车辆节点在图中标识出来
%         hold on
%     end
%     %绘制刻度尺
%     LinmapRm=R*Linmap1m;    
%     plot([20,20+LinmapRm],[80,80],'Linewidth',2);
%     text(30+LinmapRm/20,65,'200')
%     hold on
    
    PDR_set = [];%%初始化PDR
    store_road = [];%初始化store_road用来存储已遍历路段的junction点序号，junction点序号分别存在1、2列中
    all_location_relay = [];%初始化all_location_relay用来存储所有中继节点信息，第1列存中继节点所在路段来时的junction点序号，第2列存中继节点所在路段去往的junction点序号，第3、4列存中继节点的坐标
    nowlayer = 0;%初始化当前层数为0
    last_relay = branchset{2,1}(1,2:3);
    t_relay = 0;
    roadname = '1012';
    junction_time = locationjunction;%junction_time前3列等于locationjunction
    junction_time(:,4) = 0;%junction_time第4列存储该junction点的打点时间
    junction_time(:,5) = 0;%junction_time第5列存储在该junction点处打点的广播来时路段
    %车辆信息生成完毕,接下来开始提取道路信息
    store_end = zeros(24,4);%用来存储终结junction点处的last_relay信息，包括last_relay的坐标和至此的时间，第1列是存的终结junction点序号，第2、3列存的是last_relay的坐标，第4列存的是时间  
    store_wait = zeros(24,5);%用来存储等待junction点处的last_relay信息，包括last_relay的坐标和至此的时间，第1列是存的等待junction点序号，第2、3列存的是打卡消息的last_relay的坐标，第4列存的是打卡时间,第5列存的是打卡消息来时的junction点序号
    tree_root = cell(2,1);
    tree_layer1 = cell(2,2);
    tree_layer2 = cell(2,4);
    tree_layer3 = cell(2,6);
    tree_layer4 = cell(2,5);
    tree_layer5 = cell(2,7);
    tree_layer6 = cell(2,4);          
    relayinf = zeros(1,5);%第1，2列存last_relay的坐标，第3、4列存其所在路段，第5列存其打卡时间    
    [tree,endtree,waittree] = newtree;
    
    while(1)
        for i = 1:length(nowlayer)
            if nowlayer(i)==0 && isempty(tree_root{1,1})%如果在第0层（根层），且tree_root{1,1}为空，说明这才刚进入消息广播
                [nextjunction,junction_time,last_relay,t_relay,store_wait,store_end,end_relay_junction,store_road,all_location_relay,~,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,last_relay,R,Linmap1m,roadname,junction_time,t_relay,store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
            elseif nowlayer(i)==0 && isempty(tree_root{1,1})==0%如果在第0层（根层），且tree_root{1,1}不为空，说明根层路段已被广播
                roadname = [num2str(tree_root{1,1}(1,3),'%02d'),num2str(tree_root{1,1}(1,4),'%02d')];
                [nextjunction,junction_time,last_relay,t_relay,store_wait,store_end,end_relay_junction,store_road,all_location_relay,~,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_root{1,1}(1,1:2),R,Linmap1m,roadname,junction_time,tree_root{1,1}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
            elseif nowlayer(i)==1%从第一层开始出现分支了，需要考虑分支的情况，根据我们自己做的树图，得知分支为2
                roadname = [num2str(tree_layer1{1,i}(1,3),'%02d'),num2str(tree_layer1{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer1{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer1{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            elseif nowlayer(i)==2
                roadname = [num2str(tree_layer2{1,i}(1,3),'%02d'),num2str(tree_layer2{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer2{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer2{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            elseif nowlayer(i)==3
                roadname = [num2str(tree_layer3{1,i}(1,3),'%02d'),num2str(tree_layer3{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer3{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer3{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            elseif nowlayer(i)==4
                roadname = [num2str(tree_layer4{1,i}(1,3),'%02d'),num2str(tree_layer4{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer4{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer4{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            elseif nowlayer(i)==5
                roadname = [num2str(tree_layer5{1,i}(1,3),'%02d'),num2str(tree_layer5{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer5{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer5{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            elseif nowlayer(i)==6
                roadname = [num2str(tree_layer6{1,i}(1,3),'%02d'),num2str(tree_layer6{1,i}(1,4),'%02d')];
                [nextjunction_part,junction_time,last_relay_part,t_relay_part,store_wait,store_end,end_relay_junction_part,store_road,all_location_relay,PDR_part,location_vehi,num_vehi]...
                    = cell_to_singleroad(tree,endtree,waittree,locationjunction,tree_layer6{1,i}(1,1:2),R,Linmap1m,roadname,junction_time,tree_layer6{1,i}(1,5),store_wait,store_end,store_road,all_location_relay,location_vehi,num_vehi,N_iter,N_part);
                end_relay_junction = [end_relay_junction,end_relay_junction_part];
                nextjunction = [nextjunction;nextjunction_part];
                t_relay = [t_relay,t_relay_part];
                last_relay = [last_relay;last_relay_part];
                PDR_set = [PDR_set;PDR_part];
            end
        end
        nowlayer = [];%用完计层参数，就清空该参数
        
        if any(any(nextjunction))==0 || isempty(nextjunction)%如果nextjunction中所有元素都为0或者nextjunction为空          
            break;%跳出while(1)循环
        end        
        
        for j = 1:length(nextjunction(:,1))
            for i = 1:length(nextjunction(j,:))
                if nextjunction(j,i)==0
                    break;
                end
                [locationmark] = findroad(nextjunction(j,i),end_relay_junction(j),branchset,last_relay(j,1:2),locationjunction);
                if isbeacon == 1
                    [t,location_relay,relay,location_vehi,num_vehi,PDR_part]=jyc_beacon_relay_selection(location_vehi(1:num_vehi,1:4),locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay(j));
                elseif N_part == 4
                    [t,location_relay,relay,PDR_part,location_vehi,num_vehi]=jyc_bar_complete_relay_selection(location_vehi(1:num_vehi,1:4),locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay(j));%考虑障碍物的中继节点选择
                elseif N_part == 3
                    [t,location_relay,relay,PDR_part,location_vehi,num_vehi]=jyc_bar_3P3B_based_relay_selection(location_vehi(1:num_vehi,1:4),locationmark,R,num_vehi,N_iter,N_part,A,Linmap1m,T,ima2,t_relay(j));%考虑障碍物的中继节点选择
                end
                %以下将location_relay中的中继节点信息加上路段两端junction序号，调整好位置存回location_relay数组中                
                PDR_set = [PDR_set;PDR_part];
                location_relay(:,3:4)=location_relay(:,2:3);
                location_relay(:,1) = end_relay_junction(j);
                location_relay(:,2) = nextjunction(j,i);
                location_relay(location_relay(:,3)==0,:)=[];
                %以上调整location_relay数组完毕
                all_location_relay = [all_location_relay;location_relay];%将location_relay数组内容更新加进all_location_relay数组中
                relayinf(1,1:2) = relay;
                relayinf(1,3) = end_relay_junction(j);
                relayinf(1,4) = nextjunction(j,i);
                relayinf(1,5) = t;
                
                if PDR_part==0%如果消息传播失败，则不用继续传播了9.16
                    break;
                end
                
                if end_relay_junction(j)==10 && nextjunction(j,i)==12
                    tree_root{1,1} = relayinf;
                    nowlayer = [nowlayer 0];%表示在树的根层
                elseif end_relay_junction(j)==12 && nextjunction(j,i)==15
                    tree_layer1{1,1} = relayinf;
                    nowlayer = [nowlayer 1];%表示在树的第一层
                elseif end_relay_junction(j)==12 && nextjunction(j,i)==13
                    tree_layer1{1,2} = relayinf;
                    nowlayer = [nowlayer 1];%表示在树的第一层
                elseif end_relay_junction(j)==15 && nextjunction(j,i)==2
                    tree_layer2{1,1} = relayinf;
                    nowlayer = [nowlayer 2];%表示在树的第二层
                elseif end_relay_junction(j)==15 && nextjunction(j,i)==16
                    tree_layer2{1,2} = relayinf;
                    nowlayer = [nowlayer 2];%表示在树的第二层
                elseif end_relay_junction(j)==13 && nextjunction(j,i)==14
                    tree_layer2{1,3} = relayinf;
                    nowlayer = [nowlayer 2];%表示在树的第二层
                elseif end_relay_junction(j)==13 && nextjunction(j,i)==17
                    tree_layer2{1,4} = relayinf;
                    nowlayer = [nowlayer 2];%表示在树的第二层
                elseif end_relay_junction(j)==16 && nextjunction(j,i)==1
                    tree_layer3{1,1} = relayinf;
                    nowlayer = [nowlayer 3];%表示在树的第三层
                elseif end_relay_junction(j)==16 && nextjunction(j,i)==11
                    tree_layer3{1,2} = relayinf;
                    nowlayer = [nowlayer 3];%表示在树的第三层
                elseif end_relay_junction(j)==14 && nextjunction(j,i)==3
                    tree_layer3{1,3} = relayinf;
                    nowlayer = [nowlayer 3];%表示在树的第三层
                elseif end_relay_junction(j)==14 && nextjunction(j,i)==20
                    tree_layer3{1,4} = relayinf;
                    nowlayer = [nowlayer 3];%表示在树的第三层
                elseif end_relay_junction(j)==17 && nextjunction(j,i)==9
                    tree_layer3{1,5} = relayinf;
                    nowlayer = [nowlayer 3];%表示在树的第三层
                elseif end_relay_junction(j)==17 && nextjunction(j,i)==18
                    tree_layer3{1,6} = relayinf;
                    nowlayer = [nowlayer 3];%表示在树的第三层
                elseif end_relay_junction(j)==20 && nextjunction(j,i)==21
                    tree_layer4{1,1} = relayinf;
                    nowlayer = [nowlayer 4];%表示在树的第四层
                elseif end_relay_junction(j)==20 && nextjunction(j,i)==18
                    tree_layer4{1,2} = relayinf;
                    nowlayer = [nowlayer 4];%表示在树的第四层
                elseif end_relay_junction(j)==18 && nextjunction(j,i)==19
                    tree_layer4{1,3} = relayinf;
                    nowlayer = [nowlayer 4];%表示在树的第四层
                elseif end_relay_junction(j)==18 && nextjunction(j,i)==20
                    tree_layer4{1,4} = relayinf;
                    nowlayer = [nowlayer 4];%表示在树的第四层
                elseif end_relay_junction(j)==18 && nextjunction(j,i)==22
                    tree_layer4{1,5} = relayinf;
                    nowlayer = [nowlayer 4];%表示在树的第四层
                elseif end_relay_junction(j)==21 && nextjunction(j,i)==4
                    tree_layer5{1,1} = relayinf;
                    nowlayer = [nowlayer 5];%表示在树的第五层
                elseif end_relay_junction(j)==21 && nextjunction(j,i)==23
                    tree_layer5{1,2} = relayinf;
                    nowlayer = [nowlayer 5];%表示在树的第五层
                elseif end_relay_junction(j)==19 && nextjunction(j,i)==6
                    tree_layer5{1,3} = relayinf;
                    nowlayer = [nowlayer 5];%表示在树的第五层
                elseif end_relay_junction(j)==19 && nextjunction(j,i)==23
                    tree_layer5{1,4} = relayinf;
                    nowlayer = [nowlayer 5];%表示在树的第五层
                elseif end_relay_junction(j)==19 && nextjunction(j,i)==24
                    tree_layer5{1,5} = relayinf;
                    nowlayer = [nowlayer 5];%表示在树的第五层
                elseif end_relay_junction(j)==22 && nextjunction(j,i)==8
                    tree_layer5{1,6} = relayinf;
                    nowlayer = [nowlayer 5];%表示在树的第五层
                elseif end_relay_junction(j)==22 && nextjunction(j,i)==24
                    tree_layer5{1,7} = relayinf;
                    nowlayer = [nowlayer 5];%表示在树的第五层
                elseif end_relay_junction(j)==23 && nextjunction(j,i)==5
                    tree_layer6{1,1} = relayinf;
                    nowlayer = [nowlayer 6];%表示在树的第六层
                elseif end_relay_junction(j)==23 && nextjunction(j,i)==19
                    tree_layer6{1,2} = relayinf;
                    nowlayer = [nowlayer 6];%表示在树的第六层
                elseif end_relay_junction(j)==24 && nextjunction(j,i)==7
                    tree_layer6{1,3} = relayinf;
                    nowlayer = [nowlayer 6];%表示在树的第六层
                elseif end_relay_junction(j)==24 && nextjunction(j,i)==22
                    tree_layer6{1,4} = relayinf;
                    nowlayer = [nowlayer 6];%表示在树的第六层
                end               
            end   
            
            if PDR_part==0%如果消息传播失败，则不用继续传播了9.16
                break;
            end
            
        end       
        end_relay_junction = [];
        nextjunction = [];
        t_relay = [];
        last_relay = [];     
        
        if PDR_part==0%如果消息传播失败，则不用继续传播了9.16
            break;
        end
   
    end

    for j = 1:length(all_location_relay(:,1))%for循环中的代码目的是：将互为相反方向的消息传播方向给找出来，并统一为一个方向        
        %思想：第一步，从all_location_relay所有行中找出，第二列等于all_location_relay(j,1)并且第一列等于all_location_relay(j,2)的行；
        %第二步，对其在Juntion_time中的打点时间进行判定，认定消息传播方向是从打点早的点广播至打点晚的点，故根据两个点的打点时间，最终将互为相反的两个消息传播方向统一为一个消息传播方向
        [index_a,~] = find(all_location_relay(:,2)==all_location_relay(j,1));%找出all_location_relay中第二列等于all_location_relay(j,1)的行下标
        [index_b,~] = find(all_location_relay(:,1)==all_location_relay(j,2));%找出all_location_relay中第一列等于all_location_relay(j,2)的行下标
        index = intersect(index_a,index_b);
        if isempty(index)==0 && any(all_location_relay(index(1),:))~=0%如果index不为空,且index对应行里面不全为0
            index_a_used_in_amend = [];
            index_b_used_in_amend = [];
            index_used_in_amend = [];
            if all_location_relay(j,1)~=0
                if junction_time(all_location_relay(j,1),4) > junction_time(all_location_relay(j,2),4)%如果第all_location_relay(j,1)号junction点的打点时间晚于第all_location_relay(j,2)号junction的打点时间
                    [index_a_used_in_amend,~] = find(all_location_relay(:,1)==all_location_relay(j,1));
                    [index_b_used_in_amend,~] = find(all_location_relay(:,2)==all_location_relay(j,2));
                    index_used_in_amend = intersect(index_a_used_in_amend,index_b_used_in_amend);
                    all_location_relay(index_used_in_amend,:)=0;
                else                                                                                  %如果第all_location_relay(j,1)号junction点的打点时间早于第all_location_relay(j,2)号junction的打点时间
                    [index_a_used_in_amend,~] = find(all_location_relay(:,1)==all_location_relay(j,2));
                    [index_b_used_in_amend,~] = find(all_location_relay(:,2)==all_location_relay(j,1));
                    index_used_in_amend = intersect(index_a_used_in_amend,index_b_used_in_amend);
                    all_location_relay(index_used_in_amend,:)=0;
                end
            end
        end
    end
    all_location_relay(all_location_relay(:,1)==0,:)=[];
    %        通过下面的for循环在图中画出relay点
%     for j =1:length(all_location_relay(:,1))%将所有选出的中继节点在图中画出
%         plot(all_location_relay(j,3),all_location_relay(j,4),'^','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',6);
%         hold on
%     end
    PDR = mean(PDR_set(:,1));%计算PDR
    if PDR~=1%考虑端到端的PDR
        PDR = 0;
        time = NaN;%端到端消息传播失败，延时不计入其中
        maxhop = NaN;
    else%如果PDR等于1，说明端到端传输成功了，在这个else里面计算maxhop和time
        %以下开始跳数计算
        hop = 0;
        temp_all_relay = all_location_relay;
        temp_all_relay(:,5) = 0;
        while(1)
            while(1)
                [sameroad_relay_index_a,~] = find(temp_all_relay(:,1)==temp_all_relay(1,1));
                [sameroad_relay_index_b,~] = find(temp_all_relay(:,2)==temp_all_relay(1,2));
                sameroad_relay_index = intersect(sameroad_relay_index_a,sameroad_relay_index_b);
                for n = 1:length(sameroad_relay_index)
                    if hop > 0
                    %在该if语句中，我们将以下两种情况的跳数更正：
                    %一，在某路段中没有进行中继节点选择就完成了该路段的覆盖时，我们在函数中通常会将上一路段最后选择的中继节点作为此路段的中继节点，然而现实中该路段其实根本没进行中继节点选择（此情况常见于21-04路段）
                    %二，在某junction点处产生消息广播冲突，此时从该junction点多个方向来的消息广播，会重复地选择一个最靠近该junction点的车辆作为中继节点，然而其实重复的这一跳在现实中并没有必要跳（此情况常见于21号junction点）
                    %上述两种情况，其实都会导致跳数计算多出一跳，于是在if语句中，我们判断碰到上述两种情况，我们就将跳数减一，以应对此类问题。
                        [index_last_hop,~] = find(temp_all_relay(:,5) == hop);
                        if isempty(index_last_hop)==0
                            for i = 1:length(index_last_hop)
                                if index_last_hop(i) ~= 1 && temp_all_relay(index_last_hop(i),3) == temp_all_relay(1,3) && temp_all_relay(index_last_hop(i),4) == temp_all_relay(1,4)
                                    hop = hop-1;
                                    break;
                                end
                            end
                        end
                    end
                    hop = hop+1;%更新跳数
                    if temp_all_relay(sameroad_relay_index(n),5)<hop
                        temp_all_relay(sameroad_relay_index(n),5)=hop;
                    end
                end                
                [index_a_used_in_amend,~] = find(temp_all_relay(:,1) == temp_all_relay(1,2),1,'first');
                [index_b_used_in_amend,~] = find(temp_all_relay(:,2) == temp_all_relay(1,1));
                index = setdiff(index_a_used_in_amend,index_b_used_in_amend);
                if isempty(index) || junction_time(temp_all_relay(1,2),5) ~= temp_all_relay(1,1)%如果该条道路上的relay点已经覆盖到终点或者覆盖到已经被覆盖过的junction点，那么将不继续往下计算跳数
                    break;
                end
                if temp_all_relay(index,5)<=hop
                    temp_one_relay = temp_all_relay(1,:);
                    temp_all_relay(1,:) = temp_all_relay(index,:);
                    temp_all_relay(index,:) = temp_one_relay;
                else
                    break;
                end
            end
            count = 1;%初始化计数参数，用来对下面的while进行计数以免出现死循环
            while(1)               
                [index,~] = find(temp_all_relay(:,5) == 0,1,'first');%找出对应还未计算跳数的路段
                if isempty(index)%如果找不到，说明所有行都计算了跳数
                    break;
                end
                [pro_index,~] = find(temp_all_relay(:,2)==temp_all_relay(index,1));%找出对应路段的前置路段
                
                if isempty(pro_index)%如果找不出对应路段的前置路段，则说明本次消息广播是错误的，记录一下
                    PDR = 0;
                    time = NaN;%端到端消息传播失败，延时不计入其中
                    maxhop = NaN;
                    look_sign = 1;
                end
                
                if temp_all_relay(pro_index(1),5)==0%如果前置路段也未曾计算跳数
                    temp_one_relay = temp_all_relay(index,:);%互换对应路段和其前置路段在数组中的位置
                    temp_all_relay(index,:) = temp_all_relay(pro_index(1),:);
                    temp_all_relay(pro_index(1),:) = temp_one_relay;
                else
                    hop = max(temp_all_relay(pro_index,5));%矫正跳数为对应路段前置路段的储存跳数
                    temp_one_relay = temp_all_relay(1,:);%互换对应路段与数组中第一行路段的位置
                    temp_all_relay(1,:) = temp_all_relay(index,:);
                    temp_all_relay(index,:) = temp_one_relay;
                    break;
                end
                
                count = count+1;
                if count>50%如果count数大于50，意味着出现了死循环，我们需要对temp_all_relay数组中存储的起点终点位置进行修改                   
                    for k = 1:length(temp_all_relay(:,1))
                        if temp_all_relay(k,5)==0
                            if temp_all_relay(k,2)==junction_time(temp_all_relay(k,1),5)%如果temp_all_relay数组中k行存储的junction去往的终点等于junction_time数组中该junction存储的来时的点，则说明这俩点构成的路段出现了循环计数的情况，以junction_time数组中存储的为准
                                %在此if语句中调换temp_junction数组中该行起点和终点的位置
                                temp_junction = temp_all_relay(k,1);
                                temp_all_relay(k,1) = temp_all_relay(k,2);
                                temp_all_relay(k,2) = temp_junction;
                            end
                        end
                    end
                    count = 1;%还原count的计数
                end
                
            end
            if isempty(find(temp_all_relay(:,5)==0, 1))
                break;
            end
        end
        maxhop = max(temp_all_relay(:,5));
        %跳数计算结束
        time=max(junction_time(:,4));%计算端到端时延
    end    
        
end