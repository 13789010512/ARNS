function [left_dev_ang,right_dev_ang,facebar_sign]=getdevang(relay,locationbar,Linmap1m,locationmark,send_end,R)
    %这是求考虑障碍物情况relay广播时的左右角度偏移量的函数，每个障碍物为矩形且由四个障碍物标记点构成，它们的序号从图中的左侧到右侧依次递增，最左侧为第一障碍物
    %bar_num = length(locationbar)/4;%求出障碍物的个数为bar_num个，因为障碍物是长方形，由四个障碍物标记点构成，障碍物标记点都存在locationbar数组中
    %bar_around_relay = zeros(2,1);%初始化bar_around_relay数组,该数组所存内容为relay点左右两侧的障碍物序号
    facebar_sign = 0;%初始化迎面障碍物情况标记为0
    left_ang_bar_relay = [];%先初始化left_ang_bar_relay和right_ang_bar_relay数组为空
    right_ang_bar_relay = [];%之后再往两个数组里面加东西
    [relay_road_index]=jyc_find_relayroad(locationmark,relay);%求出relay所在路段
    [relay_arc,~]= cart2pol(locationmark(relay_road_index,2)-relay(2),locationmark(relay_road_index,1)-relay(1));%求出relay所在路段的对应路段点与relay点连线的弧度
    %K = tan(relay_arc);%求出relay所在路段K值（斜率值）
    relay_ang = relay_arc*180/pi;%求出relay所在路段角度°    角度应该包含了中继节点选择的方向，我们不用再做方向考量
    % if length(send_end)==2
    relay_locationbar_dis = zeros(length(locationbar(:,1)),2);%初始化relay_locationbar_dis数组，用来存放中继节点与障碍物标记点的距离，第一列存序号，第二列存距离
    for i = 1:length(locationbar(:,1))
        relay_locationbar_dis(i,1)=i;%第一列用来存序号
        relay_locationbar_dis(i,2)=(sum((relay(1:2)-locationbar(i,1:2)).^2))^0.5/Linmap1m;%第二列用来存距离，求出障碍物各标记点与relay的距离，对应位置地存入relay_locationbar_dis数组中
    end
    
    relay_locationbar_dis((relay_locationbar_dis(:,2)>R),:)=[];%删掉在通信范围R外的障碍物标记点
    %接下来求各标记点与relay构成直线的角度，然后取角度与relay所在路段角度°相减绝对值最小的障碍物标记点为我们考虑的障碍物点
    %K_bar_relay = zeros(length(relay_locationbar_dis(:,1)),2);%初始化储存障碍物和中继节点连线斜率的数组K_bar_relay，第一列存对应locationbar中的序号，第二列存K值
    ang_bar_relay = zeros(length(relay_locationbar_dis(:,1)),3);%初始化储存障碍物和中继节点连线角度的数组ang_bar_relay
    for i = 1:length(relay_locationbar_dis(:,1))
        ang_bar_relay(i,1) = relay_locationbar_dis(i,1);
        [relay_bar_arc,~]= cart2pol(locationbar(relay_locationbar_dis(i,1),2)-relay(2),locationbar(relay_locationbar_dis(i,1),1)-relay(1));
        ang_bar_relay(i,2) = relay_bar_arc*180/pi;
        ang_bar_relay(i,3) = ceil(ang_bar_relay(i,1)/4);
    end
    
    %接下来想判断relay是否出现了迎面有一座障碍物挡住的情况
    [min_index,~] = find(relay_locationbar_dis==min(relay_locationbar_dis(:,2)));
    min_dis(1:2) = relay_locationbar_dis(min_index,1:2);%找到最近的障碍物标记点，那么这个标记点所属障碍物自然是距relay最近的障碍物
    bar_name = ceil(min_dis(1)/4);%求出该最近障碍物的编号，障碍物编号从左至右分别是1,2,3,4,5...号障碍物
    j = 1;
    for i = 1:length(ang_bar_relay(:,1))
        if ceil(ang_bar_relay(i,1)/4) == bar_name
            temp_angvalue(j,1:2) = ang_bar_relay(i,1:2);
            j = j+1;
        end
    end
    if isempty(find(temp_angvalue(:,2)>relay_ang, 1))~=1 && isempty(find(temp_angvalue(:,2)<relay_ang, 1))~=1%如果temp_angvalue中既存在大于relay_ang的角度又存在小于relay_ang的角度，那么说明relay所在路段直线直接穿过了该障碍物
        %如果进入了这个if语句，说明relay迎面有一个障碍物挡住了它广播消息
        max_ang = max(temp_angvalue(:,2));
        min_ang = min(temp_angvalue(:,2));%这个障碍物对于relay的阻碍范围角度是从min_ang到max_ang
        left_dev_ang = max_ang-relay_ang;%向左偏移度数
        right_dev_ang = abs(min_ang-relay_ang);%向右偏移度数
        facebar_sign = 1;%迎面障碍物标记置1，出现这种情况，relay通信时排除relay所在直线向左偏移left_dev_ang°，向右偏移right_dev_ang°的范围
        
    else        
        left_index = 1;
        right_index = 1;
        for i = 1:length(ang_bar_relay(:,1))
            if ang_bar_relay(i,2)-relay_ang>0%求出通信范围内各个障碍物标记点与relay构成直线的角度减去relay所在路段的角度后的角度偏移量，如果偏移量大于0，则存入左向偏移数组中
                left_ang_bar_relay(left_index,1) = ang_bar_relay(i,1);
                left_ang_bar_relay(left_index,2) = ang_bar_relay(i,2)-relay_ang;%左向角度偏移量存入左向数组中
                left_index = left_index+1;
            else %求出通信范围内各个障碍物标记点与relay构成直线的角度减去relay所在路段的角度后的角度偏移量，如果偏移量小于0，则存入右向偏移数组中
                right_ang_bar_relay(right_index,1) = ang_bar_relay(i,1);
                right_ang_bar_relay(right_index,2) = abs(ang_bar_relay(i,2)-relay_ang);%右向角度偏移量存入右向数组中
                right_index = right_index+1;
            end
        end
        if isempty(left_ang_bar_relay)
            left_dev_ang = 360-max(right_ang_bar_relay(:,2));
        else
            left_dev_ang = min(left_ang_bar_relay(:,2));%从左向偏移量数组中取出最小的偏移角度作为左向偏移角度
        end
        if isempty(right_ang_bar_relay)
            right_dev_ang = 360-max(left_ang_bar_relay(:,2));
        else
            right_dev_ang = min(right_ang_bar_relay(:,2));%从右向偏移量数组中取出最小的偏移角度作为右向偏移角度
        end
        
     %   if (relay_ang+left_dev_ang > min(ang_bar_relay(:,2)) && relay_ang+left_dev_ang < max(ang_bar_relay(:,2))) || (relay_ang-right_dev_ang > min(ang_bar_relay(:,2)) && relay_ang-right_dev_ang < max(ang_bar_relay(:,2)))
            
            
     %  end
        
    end
    
end