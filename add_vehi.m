function [location_vehi,num_vehi,relay]=add_vehi(locationmark,relay,Linmap1m,ima2,posi_opt,send_end,R,num_vehi,location_vehi,old_sender,old_posi_opt)
%本函数只用于relay所在路段编号大于send_end（即，没有反向广播），且relay在junction点的R/2距离之外的情况（即，没有跳出while循环）
global locationjunction;
isjunction=0;%posi_opt是否junction点判断，0表示不是，1表示是
suc = 0;%成功标志，初始化为0
for i = 1:length(locationjunction(:,1))
    if posi_opt(1) == locationjunction(i,2) && posi_opt(2) == locationjunction(i,3)
        isjunction=1;
        break; 
    end
end
[relay_road_index]=jyc_find_relayroad(locationmark,relay);%求出relay所在路段编号
[posi_opt_road_index]=jyc_find_relayroad(locationmark,posi_opt);%求出posi_opt所在路段编号
dis_relay_Popt = (sum((relay-posi_opt).^2))^0.5/Linmap1m;
if dis_relay_Popt>R
    dis_relay_Popt = R;
end

while (suc==0)
    if isjunction==0 %如果posi_opt不是Junction点
        if dis_relay_Popt<((3/4)*R)%posi_opt距relay的距离远小于R
            relay = posi_opt;%那么直接选该点即可
            location_vehi(num_vehi+1,1) = num_vehi+1;%并将其添加进location_vehi数组中
            location_vehi(num_vehi+1,2:3) = posi_opt;
            location_vehi(num_vehi+1,4) = 0;
            num_vehi = num_vehi+1;
            break;%直接跳出while循环
        else%如果posi_opt距relay的距离接近于R，那么为防止直接选Popt点导致基于信标的算法PDR为0的次数过多，我们需要随机生成一个车辆节点
            if isempty(old_sender)==0%如果old_sender不为空，则本次sender（又称作老relay）的通信范围内随机生成的车辆不能在上一个sender（又称作old_sender）的通信范围内
                dis_old_relay_old_sender = (sum((relay-old_sender).^2))^0.5/Linmap1m;%求出老relay（又称作新sender）和老sender之间的距离
                %接下来，求出老sender的通信范围
                dis_old_Popt_old_sender = (sum((old_posi_opt-old_sender).^2))^0.5/Linmap1m;
                if dis_old_Popt_old_sender>R
                    dis_old_Popt_old_sender = R;
                end
                %老sender的通信范围求出后，在下面一行代码求出old_sender的过剩覆盖范围
                det_dis = dis_old_Popt_old_sender-dis_old_relay_old_sender;%求出old_sender的过剩覆盖范围
                if det_dis<0%排除一些微小的错误情况
                    det_dis = 0;
                end
                if dis_relay_Popt-det_dis<1 %如果old_sender的过剩覆盖范围与当前sender（即老relay）的覆盖范围相差小于1，那么直接选当前sender的最优位置生成车辆节点作为新的relay点
                    dis_new_vehi = dis_relay_Popt;   %直接选最优位置距离生成车辆节点
                else
                    dis_new_vehi = dis_relay_Popt-unidrnd(fix((dis_relay_Popt-det_dis)));   %在det_dis到dis_relay_Popt之间随机一个距离，即随机出来的车辆不能在old_sender的通信范围内，不然的话，old_sender就可以与其通信，为何要到现在才与其通信，这不合理
                end
            else
                dis_new_vehi = unidrnd(fix(dis_relay_Popt));   %在1到dis_relay_Popt之间随机一个距离
            end
        end
    else%如果posi_opt是junction点，说明我们离junction点的距离是在R/2到R之间，因为离junction点的距离在R/2以内的情况不能进入本函数
        dis_new_vehi = dis_relay_Popt-unidrnd(fix(dis_relay_Popt/2));%随机一个在dis_relay_Popt/2到dis_relay_Popt之间的较大的距离，其目的是为了保证在本次relay选出后，绝对可以跳出中继节点选择函数的while循环，以防止在此情况耽误更多跳数。因为如果此跳relay距离过小，那么明明可以在上跳之前就直接选中本跳小距离relay，我们却没有选，因为那时候小距离relay还未生成，在现实中这样的情况不合常理
    end
    if relay_road_index>send_end(3)           %relay所在路段编号大于send_end所在路段编号
        for i = 1:length(locationmark(:,1))
            if (sum((locationmark(i,1:2)-relay).^2))^0.5/Linmap1m<=dis_new_vehi && relay_road_index>i && i>=posi_opt_road_index%从路段编号数组中找出距relay的距离小于等于dis_new_vehi（从send_end开始往relay方向，第一个进来的locationmark点），且编号小于relay_road_index大于等于posi_opt_road_index的路段标记点
                if i>1
                    %开始在i路段抽样，找出该路段上距relay距离为dis_new_vehi左右的点temprelay
                    disloc = (sum((locationmark(i,1:2)-relay).^2))^0.5/Linmap1m;
                    sample = 100;
                    detX = (locationmark(i-1,1)-locationmark(i,1))/sample;
                    detY = (locationmark(i-1,2)-locationmark(i,2))/sample;
                    for j = 1:sample
                        temprelay(1,1) = locationmark(i,1)+j*detX;
                        temprelay(1,2) = locationmark(i,2)+j*detY;
                        dis_temp = (sum((temprelay-relay).^2))^0.5/Linmap1m;
                        if (sum((temprelay-relay).^2))^0.5/Linmap1m>dis_new_vehi
                            break;
                        end
                    end
                else
                    temprelay(1,1) = locationmark(i,1);
                    temprelay(1,2) = locationmark(i,2);
                end
                %开始进行是否被障碍物阻碍判断
                sample_for_bar = 100;%初始化抽样次数为100
                det_X = (temprelay(1)-relay(1))/sample_for_bar;
                det_Y = (temprelay(2)-relay(2))/sample_for_bar;
                for j = 1:sample_for_bar
                    cross_bar = 0;%将穿过障碍物的标记位初始化为0
                    temp_x = relay(1)+j*det_X;
                    temp_y = relay(2)+j*det_Y;
                    x_data = ceil(temp_x);
                    y_data = ceil(temp_y);
                    if ima2(y_data,x_data) == 0    %如果坐标对应图像矩阵的像素值为0，说明顺着连线递进时碰到了障碍物，说明连线穿过了障碍物
                        cross_bar = 1;    %将穿过障碍物的标记位置1
                        break;
                    end
                end
                %是否被障碍物阻碍判断完毕
                if (sum((temprelay-relay).^2))^0.5/Linmap1m<R && cross_bar == 0%如果我们找到的路段标记点距relay的距离小于R，且没有被障碍物阻碍
                    relay = temprelay;%则我们将其作为我们的relay点
                    location_vehi(num_vehi+1,1) = num_vehi+1;%并将其添加进location_vehi数组中
                    location_vehi(num_vehi+1,2:3) = temprelay;
                    location_vehi(num_vehi+1,4) = 0;
                    num_vehi = num_vehi+1;
                    suc = 1;%成功标志置1
                    break;
                end
            end
        end
    end
    
end