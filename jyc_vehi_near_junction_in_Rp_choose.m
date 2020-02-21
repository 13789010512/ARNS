function  [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,relay,send_end,ima2)
        location_vehi_in_Rp = zeros(length(location_vehi(:,1)),4);%初始化location_vehi_in_Rp数组
        [relay_road_index]=jyc_find_relayroad(locationmark,relay);
 %筛选车辆通信范围内的中继节点          
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
            [vehi_road_index]=jyc_find_relayroad_near_junction(locationmark,location_vehi_in_Rp(i,2:3),R,Linmap1m);
            location_vehi_in_Rp(i,4) = vehi_road_index;
        end
        
        %以下段目的是为了防止中继节点的选择出现倒退回上一步，然后又前进至现在这一步的无限循环
        if relay_road_index>send_end(3)
            for i = 1:length(location_vehi_in_Rp(:,1))
                if location_vehi_in_Rp(i,4)~=-1 && location_vehi_in_Rp(i,4)>=relay_road_index  %用大于等于是为了将可能出现的在同一路段上反复无限选择的情况解决,location_vehi_in_Rp(i,4)==-1的点是超出了本道路junction的点，要用于路口场景中继节点选择
                   location_vehi_in_Rp(i,:) = 0;
                end
            end
        elseif relay_road_index<send_end(3)
            for i = 1:length(location_vehi_in_Rp(:,1))
                if location_vehi_in_Rp(i,4)~=-1 && location_vehi_in_Rp(i,4)<=relay_road_index  %用小于等于是为了将可能出现的在同一路段上反复无限选择的情况解决
                   location_vehi_in_Rp(i,:) = 0;                    
                end
            end            
        end
        %防倒退措施至此结束
        location_vehi_in_Rp(location_vehi_in_Rp(:,1)==0,:) = [];%将location_vehi_in_Rp数组中全零行去掉 


end