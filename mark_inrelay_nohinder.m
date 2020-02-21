function [locationmark_in_relay,ang_max,ang_min]=mark_inrelay_nohinder(relay,locationbar,Linmap1m,locationmark,R)
    %这是求考虑障碍物情况relay广播时的左右角度偏移量的函数，每个障碍物为矩形且由四个障碍物标记点构成，它们的序号从图中的左侧到右侧依次递增，最左侧为第一障碍物
    %bar_num = length(locationbar)/4;%求出障碍物的个数为bar_num个，因为障碍物是长方形，由四个障碍物标记点构成，障碍物标记点都存在locationbar数组中
    %bar_around_relay = zeros(2,1);%初始化bar_around_relay数组,该数组所存内容为relay点左右两侧的障碍物序号

        [relay_road_index]=jyc_find_relayroad(locationmark,relay);%求出relay所在路段
        
        [relay_arc,~]= cart2pol(locationmark(relay_road_index,2)-relay(2),locationmark(relay_road_index,1)-relay(1));%求出relay所在路段的对应路段点与relay点连线的弧度
        relay_ang = relay_arc*180/pi;%求出relay所在路段角度°    角度应该包含了中继节点选择的方向，我们不用再做方向考量

        relay_locationbar_dis = zeros(length(locationbar(:,1)),2);%初始化relay_locationbar_dis数组，用来存放中继节点与障碍物标记点的距离，第一列存序号，第二列存距离
        for i = 1:length(locationbar(:,1))
            relay_locationbar_dis(i,1)=i;%第一列用来存序号
            relay_locationbar_dis(i,2)=(sum((relay(1:2)-locationbar(i,1:2)).^2))^0.5/Linmap1m;%第二列用来存距离，求出障碍物各标记点与relay的距离，对应位置地存入relay_locationbar_dis数组中
        end
        
        relay_locationbar_dis((relay_locationbar_dis(:,2)>R),:)=[];%删掉在通信范围R外的障碍物标记点
        ang_bar_relay = zeros(length(relay_locationbar_dis(:,1)),3);%初始化储存障碍物和中继节点连线角度的数组ang_bar_relay
        for i = 1:length(relay_locationbar_dis(:,1))
            ang_bar_relay(i,1) = relay_locationbar_dis(i,1);
            [relay_bar_arc,~]= cart2pol(locationbar(relay_locationbar_dis(i,1),2)-relay(2),locationbar(relay_locationbar_dis(i,1),1)-relay(1));
            ang_bar_relay(i,2) = relay_bar_arc*180/pi;
            ang_bar_relay(i,3) = ceil(ang_bar_relay(i,1)/4);
        end
        %在这里加新方法
        locationmark_in_relay = zeros(length(locationmark(:,1)),3);%初始化储存中继节点通信范围内的路段标记点的数组，第一列存路段标记点编号，第二列存路段标记点的X值，第三列存路段标记点的Y值
        j = 1;
        for i = 1:length(locationmark(:,1))
            if (sum((locationmark(i,1:2)-relay(1:2)).^2))^0.5/Linmap1m < R
                locationmark_in_relay(j,1) = i;
                locationmark_in_relay(j,2:3) = locationmark(i,1:2);
                j = j+1;
            end
        end
        locationmark_in_relay(locationmark_in_relay(:,1)==0,:)=[];
        ang_min = zeros(ang_bar_relay(length(ang_bar_relay(:,1)),3),4);%初始化ang_min数组，行数为障碍物数，列数为4，第一列为障碍物标记点所属障碍物序号，第二列为障碍物标记点与relay的最小角度值，第三列为障碍物标记点的X值，第四列为Y值
        ang_max = zeros(ang_bar_relay(length(ang_bar_relay(:,1)),3),4);%初始化ang_max数组，行数为障碍物数，列数为4，第一列为障碍物标记点所属障碍物序号，第二列为障碍物标记点与relay的最大角度值，第三列为障碍物标记点的X值，第四列为Y值
       %下面这些参数是后加的
        ang_bar_relay_dev = zeros(ang_bar_relay(length(ang_bar_relay(:,1)),3),4);%初始化ang_bar_relay_dev数组，第一列为障碍物标记点序号，第二列为其与X轴（这里X轴是竖轴）偏移角度的值，第三列为所属障碍物编号，第四列为bar_relay连线所在直角坐标系的区间
        temp_ang_max = zeros(ang_bar_relay(length(ang_bar_relay(:,1)),3),5);%同ang_max理，只不过第二列是偏移角值，第五列是所在直角坐标系区间值
        temp_ang_min = zeros(ang_bar_relay(length(ang_bar_relay(:,1)),3),5);%同ang_min理，只不过第二列是偏移角值，第五列是所在直角坐标系区间值
        for j = 1:length(ang_bar_relay(:,1))
             %下面内容是后加的
            ang_bar_relay_dev(j,1) = ang_bar_relay(j,1);
            ang_bar_relay_dev(j,3) = ang_bar_relay(j,3);
            if ang_bar_relay(j,2) >= 0 && ang_bar_relay(j,2) < 90
                ang_bar_relay_dev(j,2) = ang_bar_relay(j,2)-0;
                ang_bar_relay_dev(j,4) = 1;%位于第一区间
            elseif ang_bar_relay(j,2) >=90 && ang_bar_relay(j,2) <= 180
                ang_bar_relay_dev(j,2) = 180-ang_bar_relay(j,2);
                ang_bar_relay_dev(j,4) = 2;%位于第二区间
            elseif ang_bar_relay(j,2) < 0 && ang_bar_relay(j,2) > -90
                ang_bar_relay_dev(j,2) = 0-ang_bar_relay(j,2);
                ang_bar_relay_dev(j,4) = 4;%位于第四区间
            elseif ang_bar_relay(j,2) <= -90 && ang_bar_relay(j,2) >= -180
                ang_bar_relay_dev(j,2) = ang_bar_relay(j,2)-(-180);
                ang_bar_relay_dev(j,4) = 3;%位于第三区间
            end                        
            %后加内容到此为止，目的为了更好的筛选出障碍物阻碍的角度范围,该for循环验证结果是正确的 
        end
        sign = 0;
        for j = 1:length(ang_bar_relay(:,1))           
            if temp_ang_max(ang_bar_relay(j,3),1)~= ang_bar_relay(j,3)%如果已经到了下一个障碍物了                
                temp_ang_max(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                temp_ang_max(ang_bar_relay(j,3),2) = ang_bar_relay_dev(j,2);
                temp_ang_max(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                temp_ang_max(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);
                sign = 0;
                [rank,~] = find(ang_bar_relay_dev(:,4)~=temp_ang_max(ang_bar_relay(j,3),5) & ang_bar_relay_dev(:,3)==temp_ang_max(ang_bar_relay(j,3),1));%从ang_bar_relay_dev数组中筛选出与temp_ang_max第ang_bar_relay(j,3)行同属一个障碍物却属于不同区间的障碍物标记点
                if isempty(rank)
                    temp_ang_min(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                    temp_ang_min(ang_bar_relay(j,3),2) = 0;
                    temp_ang_min(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                    temp_ang_min(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);
                    sign = 1;%置1表示：障碍物整体都在某一个区间内
                else
                    temp_ang_min(ang_bar_relay(j,3),1) = ang_bar_relay(rank(1),3);
                    temp_ang_min(ang_bar_relay(j,3),2) = ang_bar_relay_dev(rank(1),2);
                    temp_ang_min(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(rank(1),1),1:2);
                    temp_ang_min(ang_bar_relay(j,3),5) = ang_bar_relay_dev(rank(1),4);
                end
                %以下是整个障碍物横跨两个区间的情况  
            elseif temp_ang_max(ang_bar_relay(j,3),5) == ang_bar_relay_dev(j,4) && sign==0%对temp_ang_max来说，如果是在一个区间上，且标记为0（标记为0表示，整个障碍物横跨两个区间）
                if temp_ang_max(ang_bar_relay(j,3),2) > ang_bar_relay_dev(j,2)%如果temp_ang_max是该区间偏离X轴最多的
                    temp_ang_max(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                    temp_ang_max(ang_bar_relay(j,3),2) = ang_bar_relay_dev(j,2);
                    temp_ang_max(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                    temp_ang_max(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);%选出偏离X轴最少的放入 temp_ang_max 数组中
                end
            elseif temp_ang_min(ang_bar_relay(j,3),5) == ang_bar_relay_dev(j,4) && sign==0%对temp_ang_min来说，如果是在一个区间上，且标记为0
                if temp_ang_min(ang_bar_relay(j,3),2) > ang_bar_relay_dev(j,2)%如果temp_ang_min是该区间偏离X轴最多的
                    temp_ang_min(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                    temp_ang_min(ang_bar_relay(j,3),2) = ang_bar_relay_dev(j,2);
                    temp_ang_min(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                    temp_ang_min(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);%%选出偏离X轴最少的放入 temp_ang_min 数组中
                end
                %以下是整个障碍物都在一个区间内的情况
            elseif temp_ang_max(ang_bar_relay(j,3),5) == ang_bar_relay_dev(j,4) && sign==1%对temp_ang_max来说，如果在一个区间，且标记为1（标记为1表示整个障碍物都在一个区间内）
                if temp_ang_max(ang_bar_relay(j,3),2) < ang_bar_relay_dev(j,2)%如果temp_ang_max不是该区间偏离X轴最多的
                    temp_ang_max(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                    temp_ang_max(ang_bar_relay(j,3),2) = ang_bar_relay_dev(j,2);
                    temp_ang_max(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                    temp_ang_max(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);%选出偏离X轴最多的放入 temp_ang_max 数组中
                end
            elseif temp_ang_min(ang_bar_relay(j,3),5) == ang_bar_relay_dev(j,4) && sign==1%对temp_ang_min来说，如果在一个区间，且标记为1
                if temp_ang_min(ang_bar_relay(j,3),2) > ang_bar_relay_dev(j,2)%如果temp_ang_min是该区间偏离X轴最多的
                    temp_ang_min(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                    temp_ang_min(ang_bar_relay(j,3),2) = ang_bar_relay_dev(j,2);
                    temp_ang_min(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                    temp_ang_min(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);%%选出偏离X轴最少的放入 temp_ang_min 数组中
                end
            end%if语句到此结束
        end
        for j = 1:length(temp_ang_max(:,1))
            %以下开始判断赋值环节
            if (temp_ang_max(j,5) == 1 && temp_ang_min(j,5) == 4)  ||  (temp_ang_max(j,5) == 1 && temp_ang_min(j,5) == 3 && (temp_ang_max(j,2)-temp_ang_min(j,2))<0)
               ang_max(j,1:4) = temp_ang_max(j,1:4);
               if temp_ang_min(j,5) == 4
                   ang_min(j,1) = temp_ang_min(j,1);
                   ang_min(j,2) = 360-temp_ang_min(j,2);
                   ang_min(j,3:4) = temp_ang_min(j,3:4);
               elseif temp_ang_min(j,5) == 3
                   ang_min(j,1) = temp_ang_min(j,1);
                   ang_min(j,2) = 180+temp_ang_min(j,2);
                   ang_min(j,3:4) = temp_ang_min(j,3:4);
               end
            elseif temp_ang_max(j,5) == 2 && temp_ang_min(j,5) == 1  ||  (temp_ang_max(j,5) == 2 && temp_ang_min(j,5) == 4 && (temp_ang_max(j,2)-temp_ang_min(j,2))>0)
               ang_max(j,1) = temp_ang_max(j,1);
               ang_max(j,2) = 180-temp_ang_max(j,2);
               ang_max(j,3:4) = temp_ang_max(j,3:4);
               if temp_ang_min(j,5) == 1
                   ang_min(j,1:4) = temp_ang_min(j,1:4);
               elseif  temp_ang_min(j,5) == 4
                   ang_min(j,1) = temp_ang_min(j,1);
                   ang_min(j,2) = 360-temp_ang_min(j,2);
                   ang_min(j,3:4) = temp_ang_min(j,3:4);
               end
            elseif temp_ang_max(j,5) == 3 && temp_ang_min(j,5) == 2  ||  (temp_ang_max(j,5) == 3 && temp_ang_min(j,5) == 1 && (temp_ang_max(j,2)-temp_ang_min(j,2))<0)
               ang_max(j,1) = temp_ang_max(j,1);
               ang_max(j,2) = 180+temp_ang_max(j,2);
               ang_max(j,3:4) = temp_ang_max(j,3:4); 
               if temp_ang_min(j,5) == 2
                   ang_min(j,1) = temp_ang_min(j,1);
                   ang_min(j,2) = 180-temp_ang_min(j,2);
                   ang_min(j,3:4) = temp_ang_min(j,3:4);
               elseif temp_ang_min(j,5) == 1
                   ang_min(j,1:4) = temp_ang_min(j,1:4);
               end
            elseif temp_ang_max(j,5) == 4 && temp_ang_min(j,5) == 3  ||  (temp_ang_max(j,5) == 4 && temp_ang_min(j,5) == 2 && (temp_ang_max(j,2)-temp_ang_min(j,2))>0)
               ang_max(j,1) = temp_ang_max(j,1);
               ang_max(j,2) = 360-temp_ang_max(j,2);
               ang_max(j,3:4) = temp_ang_max(j,3:4); 
               if temp_ang_min(j,5) == 3
                   ang_min(j,1) = temp_ang_min(j,1);
                   ang_min(j,2) = 180+temp_ang_min(j,2);
                   ang_min(j,3:4) = temp_ang_min(j,3:4);
               elseif temp_ang_min(j,5) == 2
                   ang_min(j,1) = temp_ang_min(j,1);
                   ang_min(j,2) = 180-temp_ang_min(j,2);
                   ang_min(j,3:4) = temp_ang_min(j,3:4);
               end
            else 
                temp = temp_ang_max;
                temp_ang_max = temp_ang_min;
                temp_ang_min = temp;                
                %互换Max和Min后继续上述判断赋值环节
                if (temp_ang_max(j,5) == 1 && temp_ang_min(j,5) == 4)  ||  (temp_ang_max(j,5) == 1 && temp_ang_min(j,5) == 3 && (temp_ang_max(j,2)-temp_ang_min(j,2))<0)
                    ang_max(j,1:4) = temp_ang_max(j,1:4);
                    if temp_ang_min(j,5) == 4
                        ang_min(j,1) = temp_ang_min(j,1);
                        ang_min(j,2) = 360-temp_ang_min(j,2);
                        ang_min(j,3:4) = temp_ang_min(j,3:4);
                    elseif temp_ang_min(j,5) == 3
                        ang_min(j,1) = temp_ang_min(j,1);
                        ang_min(j,2) = 180+temp_ang_min(j,2);
                        ang_min(j,3:4) = temp_ang_min(j,3:4);
                    end
                elseif temp_ang_max(j,5) == 2 && temp_ang_min(j,5) == 1  ||  (temp_ang_max(j,5) == 2 && temp_ang_min(j,5) == 4 && (temp_ang_max(j,2)-temp_ang_min(j,2))>0)
                    ang_max(j,1) = temp_ang_max(j,1);
                    ang_max(j,2) = 180-temp_ang_max(j,2);
                    ang_max(j,3:4) = temp_ang_max(j,3:4);
                    if temp_ang_min(j,5) == 1
                        ang_min(j,1:4) = temp_ang_min(j,1:4);
                    elseif  temp_ang_min(j,5) == 4
                        ang_min(j,1) = temp_ang_min(j,1);
                        ang_min(j,2) = 360-temp_ang_min(j,2);
                        ang_min(j,3:4) = temp_ang_min(j,3:4);
                    end
                elseif temp_ang_max(j,5) == 3 && temp_ang_min(j,5) == 2  ||  (temp_ang_max(j,5) == 3 && temp_ang_min(j,5) == 1 && (temp_ang_max(j,2)-temp_ang_min(j,2))<0)
                    ang_max(j,1) = temp_ang_max(j,1);
                    ang_max(j,2) = 180+temp_ang_max(j,2);
                    ang_max(j,3:4) = temp_ang_max(j,3:4);
                    if temp_ang_min(j,5) == 2
                        ang_min(j,1) = temp_ang_min(j,1);
                        ang_min(j,2) = 180-temp_ang_min(j,2);
                        ang_min(j,3:4) = temp_ang_min(j,3:4);
                    elseif temp_ang_min(j,5) == 1
                        ang_min(j,1:4) = temp_ang_min(j,1:4);
                    end
                elseif temp_ang_max(j,5) == 4 && temp_ang_min(j,5) == 3  ||  (temp_ang_max(j,5) == 4 && temp_ang_min(j,5) == 2 && (temp_ang_max(j,2)-temp_ang_min(j,2))>0)
                    ang_max(j,1) = temp_ang_max(j,1);
                    ang_max(j,2) = 360-temp_ang_max(j,2);
                    ang_max(j,3:4) = temp_ang_max(j,3:4);
                    if temp_ang_min(j,5) == 3
                        ang_min(j,1) = temp_ang_min(j,1);
                        ang_min(j,2) = 180+temp_ang_min(j,2);
                        ang_min(j,3:4) = temp_ang_min(j,3:4);
                    elseif temp_ang_min(j,5) == 2
                        ang_min(j,1) = temp_ang_min(j,1);
                        ang_min(j,2) = 180-temp_ang_min(j,2);
                        ang_min(j,3:4) = temp_ang_min(j,3:4);
                    end
                end %互换后的判断赋值环节结束   
                                                        
            end%总判断赋值环节结束
                   
        end%以上验证结果是正确的
        
        relay_mark_arc = 0;%初始化中继节点和路段标记点连线角度参数为0
        relay_mark_ang = 0;%初始化中继节点和路段标记点连线角度参数为0°
        ang_max(ang_max(:,1)==0,:)=[];%删除ang_max数组中的全零行
        ang_min(ang_min(:,1)==0,:)=[];%删除ang_min数组中的全零行
        if length(ang_max(:,1)) > length(ang_min(:,1))
           ang_max(length(ang_max(:,1)),:)=[];
        elseif length(ang_max(:,1)) < length(ang_min(:,1))
           ang_min(length(ang_min(:,1)),:)=[]; 
        end
        for i = 1:length(locationmark_in_relay(:,1))
            [relay_mark_arc,~]= cart2pol(locationmark_in_relay(i,3)-relay(2),locationmark_in_relay(i,2)-relay(1));%locationmark_in_relay数组中第1列为路段标记点编号，第2列为X，第3列为Y
            dis_mark_relay = (sum((locationmark_in_relay(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m;
            relay_mark_ang = relay_mark_arc*180/pi;
            for j = 1:length(ang_max(:,1))
                tempmin = zeros(1,3);%临时Min三维坐标表示，第三维是0
                tempmax = zeros(1,3);%同上
                temprelay = zeros(1,3);%同上
                if ang_max(j,2)>ang_min(j,2)
                    if relay_mark_ang<ang_max(j,2) && relay_mark_ang>ang_min(j,2)%满足该条件，说明路段点有可能被障碍物阻挡，接下来看看路段点是在障碍物和relay之间，还是在障碍物后面
                        tempmin(1,1:2) = ang_min(j,3:4);%第1、2维对应X、Y坐标
                        tempmin(1,3) = 0;%第三维是0
                        tempmax(1,1:2) = ang_max(j,3:4);
                        tempmax(1,3) = 0;
                        temprelay(1,1:2) = relay(1:2);
                        temprelay(1,3) = 0;
                        dis_relay_barline = (norm(cross(tempmin-tempmax,temprelay-tempmax))/norm(tempmin-tempmax))/Linmap1m;
                        if dis_relay_barline < dis_mark_relay %relay与障碍物边线之间的距离小于路段标记点与relay的距离，即路段点是在障碍物后面，被挡住了
                            locationmark_in_relay(i,:) = 0;%将relay范围内的路段标记点数组中该行元素全部置0，即将该路段点所有讯息化为0，该零行将在后面被清除掉
                        end
                    end
                else
                    if (relay_mark_ang>=0 && relay_mark_ang<ang_max(j,2)) || (relay_mark_ang>ang_min(j,2) && relay_mark_ang<360)%特别判断第四一区间特殊情况
                        tempmin(1,1:2) = ang_min(j,3:4);%第1、2维对应X、Y坐标
                        tempmin(1,3) = 0;%第三维是0
                        tempmax(1,1:2) = ang_max(j,3:4);
                        tempmax(1,3) = 0;
                        temprelay(1,1:2) = relay(1:2);
                        temprelay(1,3) = 0;
                        dis_relay_barline = (norm(cross(tempmin-tempmax,temprelay-tempmax))/norm(tempmin-tempmax))/Linmap1m;
                        if dis_relay_barline < dis_mark_relay %relay与障碍物边线之间的距离小于路段标记点与relay的距离，即路段点是在障碍物后面，被挡住了
                            locationmark_in_relay(i,:) = 0;%将relay范围内的路段标记点数组中该行元素全部置0，即将该路段点所有讯息化为0，该零行将在后面被清除掉
                        end                         
                    end
                end
            end
        end
        locationmark_in_relay(locationmark_in_relay(:,1)==0,:)=[];%清除掉locationmark_in_relay数组中的零行
        %新方法结束
end
    
  