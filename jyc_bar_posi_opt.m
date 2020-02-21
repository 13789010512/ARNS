    function  [posi_opt,location_vehi_in_Rp]=jyc_bar_posi_opt(location_vehi,locationmark,R,Linmap1m,relay_main,send_end,ima2)
        %该代码用于选择弯道算法和直道算法的Popt点，这里输入的send_end在正常广播下都是多行两列的，在弯道算法中的反向广播时输入的re_send_end是多行三列的
        %此时代码通过区分send_end是否有第三列存在来判断是进行反向广播的Popt选取还是正常广播的Popt选取
        %road_location_max = 0;%初始化连续路段最大标号参数为0
        posi_opt = [];%初始化posi_opt参数
        relay = relay_main;       
        location_vehi_in_Rp = zeros(length(location_vehi(:,1)),4);%初始化location_vehi_in_Rp数组，该数组的列数、行数与location_vehi保持一致，location_vehi_in_Rp数组为location_vehi数组中的节点位于Rp扇形区域中的部分                
        [relay_road_index]=jyc_find_relayroad(locationmark,relay);
        if send_end(1,3) == 1
            broadcast_sign = 2;%如果broadcast_sign为2，表示在进行正常广播，如果broadcast_sign为3便是进行的是反向广播
        else
            broadcast_sign = 3;
        end
        mark_in_R = zeros(100,3);%初始化mark_in_R数组（通信范围内的路段标记点信息数组）
        jump_mark_in_R = zeros(100,3);%初始化jump_mark_in_R数组（通信范围内路段标记点跳点数组，用来储存路段标记点的跳点信息）
        mark_in_R_index = 1;%mark_in_R数组的计数参数
        for i = 1:length(locationmark(:,1))
            if (sum((locationmark(i,1:2)-relay(1:2)).^2))^0.5/Linmap1m < R
               mark_in_R(mark_in_R_index,1) = i;
               mark_in_R(mark_in_R_index,2:3) = locationmark(i,1:2);
               mark_in_R_index = mark_in_R_index + 1;
            end
        end
        mark_in_R(mark_in_R(:,1)==0,:) = [];%删除mark_in_R数组中的全零行
        
        for i = 1:length(mark_in_R(:,1)) %此循环用来删除被障碍物阻碍的路段标记点
            sample_num = 100;%初始化抽样次数为100
            det_X = (mark_in_R(i,2)-relay(1))/sample_num;
            det_Y = (mark_in_R(i,3)-relay(2))/sample_num;            
            for j = 1:sample_num
                temp_x = relay(1)+j*det_X;
                temp_y = relay(2)+j*det_Y;
                %   plot(temp_x,temp_y,'o');
                %   hold on
                x_data = ceil(temp_x);
                y_data = ceil(temp_y);
                if ima2(y_data,x_data) == 0    %如果坐标对应图像矩阵的像素值为0，说明此处为障碍物，说明车辆被障碍物阻碍了
                    mark_in_R(i,:) = 0;    %将这一个车辆的信息全部置0
                    break;
                end
            end            
        end
        mark_in_R(mark_in_R(:,1)==0,:) = [];%删除mark_in_R数组中的全零行
        jump_mark_in_R_index = 1; %路段标记点跳点储存数组的序数初始化为1
        for i = 1:length(mark_in_R(:,1))
            if i == length(mark_in_R(:,1)) && length(mark_in_R(:,1)) ~= 1
                break;
            elseif i == length(mark_in_R(:,1)) && length(mark_in_R(:,1)) == 1%如果通信范围内只存了一个路段标记点，那么这一个路段标记点可作跳点看
                jump_mark_in_R(jump_mark_in_R_index,:) = mark_in_R(i,:);%将跳点信息存入
                jump_mark_in_R_index = jump_mark_in_R_index + 1;
                break;
            end
            if abs(mark_in_R(i,1)-mark_in_R(i+1,1)) ~= 1 %找出数组中储存的路段标记点跳点
                jump_mark_in_R(jump_mark_in_R_index,:) = mark_in_R(i,:);%将跳点信息存入
                jump_mark_in_R(jump_mark_in_R_index+1,:) = mark_in_R(i+1,:);%将跳点信息存入
                jump_mark_in_R_index = jump_mark_in_R_index + 2;%跳点数组存了两个跳点，故加2
            end
        end
        jump_mark_in_R(jump_mark_in_R_index,:)=mark_in_R(mark_in_R(:,1)==min(mark_in_R(:,1)),:);%取出mark_in_R中储存的最小编号路段标记点的信息存入跳点数组中
        jump_mark_in_R_index = jump_mark_in_R_index+1;%跳点数组序数加1
        jump_mark_in_R(jump_mark_in_R_index,:)=mark_in_R(mark_in_R(:,1)==max(mark_in_R(:,1)),:);%取出mark_in_R中储存的最大编号路段标记点的信息存入跳点数组中
        jump_mark_in_R(jump_mark_in_R(:,1)==0,:) = [];%删除跳点数组（jump_mark_in_R）中的全零行
        %这时不用跳点序数加1了，因为我们已经完成了跳点的储存，接下来要对储存的跳点重复的点进行删除
        for i = 1:length(jump_mark_in_R(:,1))
            temp_jump_mark = jump_mark_in_R;
            temp_jump_mark(i,:) = [];%设置一个没有jump_mark_in_R(i,1)信息的临时数组，从剩下的跳点信息中找看有没有和jump_mark_in_R(i,1)信息重复的跳点
            if ~isempty(find(temp_jump_mark(:,1)==jump_mark_in_R(i,1), 1))%如果在跳点数组（jump_mark_in_R）中找得到jump_mark_in_R(i,1)的重复路段标记点
                jump_mark_in_R(i,:) = 0;%则将jump_mark_in_R(i,1)的信息全部置0
            end
        end
        jump_mark_in_R(jump_mark_in_R(:,1)==0,:) = [];%删除跳点数组（jump_mark_in_R）中的全零行
        %接下来开始从跳点数组中选出中继节点最优位置所在路段的某一侧路段标记点
        for i = 1:length(jump_mark_in_R(:,1))
            if abs(jump_mark_in_R(i,1)-send_end(3)) == min(abs(jump_mark_in_R(:,1)-send_end(3)))
                posi_opt_mark = jump_mark_in_R(i,:);            
            end
        end
        %接下来根据中继节点最优位置所在路段的某一侧路段标记点以10m为递进找出中继节点最优位置posi_opt
        if posi_opt_mark(1) > send_end(3)%如果路段标记点序号大于消息传播终点所在路段
            sample_num = ceil(((sum((locationmark(posi_opt_mark(1)-1,1:2)-locationmark(posi_opt_mark(1),1:2)).^2))^0.5/Linmap1m)/10);%求出以10m递进采样，一共可采样的次数
            detX = (locationmark(posi_opt_mark(1)-1,1)-locationmark(posi_opt_mark(1),1))/sample_num;
            detY = (locationmark(posi_opt_mark(1)-1,2)-locationmark(posi_opt_mark(1),2))/sample_num;
            for i = 1:sample_num
                temppoint(1) = posi_opt_mark(2)+i*detX;
                temppoint(2) = posi_opt_mark(3)+i*detY;
                
                %接下来这一段是判断temppoint与relay的连线是否穿过障碍物，穿过则cross_bar为1，不穿过则cross_bar为0
                sample_for_bar = 100;%初始化抽样次数为100
                det_X = (temppoint(1)-relay(1))/sample_for_bar;
                det_Y = (temppoint(2)-relay(2))/sample_for_bar;
                for j = 1:sample_for_bar
                    cross_bar = 0;%将穿过障碍物的标记位初始化为0
                    temp_x = relay(1)+j*det_X;
                    temp_y = relay(2)+j*det_Y;
                    %   plot(temp_x,temp_y,'o');
                    %   hold on
                    x_data = ceil(temp_x);
                    y_data = ceil(temp_y);
                    if ima2(y_data,x_data) == 0    %如果坐标对应图像矩阵的像素值为0，说明顺着连线递进时碰到了障碍物，说明连线穿过了障碍物
                        cross_bar = 1;    %将穿过障碍物的标记位置1
                        break;
                    end
                end
                %判断结束                 
                if (sum((temppoint(1:2)-relay(1:2)).^2))^0.5/Linmap1m > R %如果超出通信范围，则说明上一位temppoint是posi_opt点
                    if i ~= 1
                        posi_opt(1) = posi_opt_mark(2)+(i-1)*detX;   %则倒回去存储上一位的坐标信息
                        posi_opt(2) = posi_opt_mark(3)+(i-1)*detY;
                    else
                        posi_opt = posi_opt_mark(2:3);
                    end
                elseif cross_bar == 1%如果temppoint与relay的连线碰到了障碍物，即cross_bar为1
                    if i ~= 1
                        posi_opt(1) = posi_opt_mark(2)+(i-1)*detX;   %则倒回去存储上一位的坐标信息
                        posi_opt(2) = posi_opt_mark(3)+(i-1)*detY;
                    else
                        posi_opt = posi_opt_mark(2:3);
                    end
                end
            end
        elseif posi_opt_mark(1) <= send_end(3)%如果路段标记点序号小于消息传播终点所在路段            
            sample_num = ceil(((sum((locationmark(posi_opt_mark(1)+1,1:2)-locationmark(posi_opt_mark(1),1:2)).^2))^0.5/Linmap1m)/10);%求出以10m递进采样，一共可采样的次数
            detX = (locationmark(posi_opt_mark(1)+1,1)-locationmark(posi_opt_mark(1),1))/sample_num;
            detY = (locationmark(posi_opt_mark(1)+1,2)-locationmark(posi_opt_mark(1),2))/sample_num;
            for i = 1:sample_num
                temppoint(1) = posi_opt_mark(2)+i*detX;
                temppoint(2) = posi_opt_mark(3)+i*detY;
                
                %接下来这一段是判断temppoint与relay的连线是否穿过障碍物，穿过则cross_bar为1，不穿过则cross_bar为0
                sample_for_bar = 100;%初始化抽样次数为100
                det_X = (temppoint(1)-relay(1))/sample_for_bar;
                det_Y = (temppoint(2)-relay(2))/sample_for_bar;
                for j = 1:sample_for_bar
                    cross_bar = 0;%将穿过障碍物的标记位初始化为0
                    temp_x = relay(1)+j*det_X;
                    temp_y = relay(2)+j*det_Y;
                    %   plot(temp_x,temp_y,'o');
                    %   hold on
                    x_data = ceil(temp_x);
                    y_data = ceil(temp_y);
                    if ima2(y_data,x_data) == 0    %如果坐标对应图像矩阵的像素值为0，说明顺着连线递进时碰到了障碍物，说明连线穿过了障碍物
                        cross_bar = 1;    %将穿过障碍物的标记位置1
                        break;
                    end
                end
                %判断结束                 
                if (sum((temppoint(1:2)-relay(1:2)).^2))^0.5/Linmap1m > R %如果超出通信范围，则说明上一位temppoint是posi_opt点
                    if i ~= 1
                        posi_opt(1) = posi_opt_mark(2)+(i-1)*detX;   %则倒回去存储上一位的坐标信息
                        posi_opt(2) = posi_opt_mark(3)+(i-1)*detY;
                    else
                        posi_opt = posi_opt_mark(2:3);
                    end
                elseif cross_bar == 1%如果temppoint与relay的连线碰到了障碍物，即cross_bar为1
                    if i ~= 1
                        posi_opt(1) = posi_opt_mark(2)+(i-1)*detX;   %则倒回去存储上一位的坐标信息
                        posi_opt(2) = posi_opt_mark(3)+(i-1)*detY;
                    else
                        posi_opt = posi_opt_mark(2:3);
                    end
                end
            end                       
        end
             
    
        %接下来将扇形区域Rp范围内的车辆节点也筛选一遍              
        for i = 1:length(location_vehi(:,1))
            sample_num = 100;%初始化抽样次数为100
            det_X = (location_vehi(i,2)-relay(1))/sample_num;
            det_Y = (location_vehi(i,3)-relay(2))/sample_num;
            if (sum((location_vehi(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m > R  %在通信范围外，将这一个车辆的信息全部置0
                location_vehi(i,:) = 0;
            else
                for j = 1:sample_num
                    temp_x = relay(1)+j*det_X;
                    temp_y = relay(2)+j*det_Y;
                 %   plot(temp_x,temp_y,'o');
                 %   hold on
                    x_data = ceil(temp_x);
                    y_data = ceil(temp_y);
                    if ima2(y_data,x_data) == 0    %如果坐标对应图像矩阵的像素值为0，说明此处为障碍物，说明车辆被障碍物阻碍了
                        location_vehi(i,:) = 0;    %将这一个车辆的信息全部置0
                        break;
                    end
                end
            end
        end
        location_vehi_in_Rp = location_vehi;
        location_vehi_in_Rp(location_vehi_in_Rp(:,1)==0,:) = [];%将location_vehi_in_Rp数组中全零行去掉   
        
        %下面这个for循环目的是，将每一个车辆节点所在路段的序号求出放入location_vehi_in_Rp数组的第四列
        for i = 1:length(location_vehi_in_Rp(:,1))   
            [vehi_road_index]=jyc_find_relayroad(locationmark,location_vehi_in_Rp(i,2:3));
            location_vehi_in_Rp(i,4) = vehi_road_index;
        end
        
        %以下段目的是为了防止中继节点的选择出现倒退回上一步，然后又前进至现在这一步的无限循环
        if relay_road_index>send_end(3)
            for i = 1:length(location_vehi_in_Rp(:,1))
                if location_vehi_in_Rp(i,4)>=relay_road_index  %用大于等于是为了将可能出现的在同一路段上反复无限选择的情况解决
                   location_vehi_in_Rp(i,:) = 0;
                end
            end
        elseif relay_road_index<send_end(3)
            for i = 1:length(location_vehi_in_Rp(:,1))
                if location_vehi_in_Rp(i,4)<=relay_road_index  %用小于等于是为了将可能出现的在同一路段上反复无限选择的情况解决
                   location_vehi_in_Rp(i,:) = 0;                    
                end
            end            
        end
        %防倒退措施至此结束
        location_vehi_in_Rp(location_vehi_in_Rp(:,1)==0,:) = [];%将location_vehi_in_Rp数组中全零行去掉 
        if isempty(location_vehi_in_Rp)==0%如果location_vehi_in_Rp数组不为空
            %找出location_vehi_in_Rp数组中重复的元素将其删除
            for i = 1:length(location_vehi_in_Rp(:,1))
                for j = 1:length(location_vehi_in_Rp(:,1))
                    if i~=j && location_vehi_in_Rp(i,2) == location_vehi_in_Rp(j,2) && location_vehi_in_Rp(i,3) == location_vehi_in_Rp(j,3)
                        location_vehi_in_Rp(j,:) = 0;
                    end
                end
            end
            location_vehi_in_Rp(location_vehi_in_Rp(:,1)==0,:) = [];%将location_vehi_in_Rp数组中全零行去掉 
        end
        
        if broadcast_sign == 2
            if isempty(posi_opt) && posi_opt_mark(1,2) == send_end(1,1) && posi_opt_mark(1,3) == send_end(1,2)%如果posi_opt为空，并且其选出的最优位置所在路段标记点（posi_opt_mark）就是send_end点，那么将send_end点定为最优位置（posi_opt）
                posi_opt = send_end(1,1:2);
            end
%             plot(posi_opt(1),posi_opt(2),'*','LineWidth',1,'MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',8);%正常广播的Popt标绿
%             hold on
        elseif broadcast_sign == 3
            posi_opt = [0 0];
        end
    end
