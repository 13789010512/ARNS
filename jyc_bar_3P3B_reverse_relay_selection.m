function [location_relay,re_t,num_vehi,location_vehi,location_re_relay,n_re_relay,has_cov_symbol,PDR,last_t]=jyc_bar_3P3B_reverse_relay_selection(location_vehi,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,send_start,t,re_t,location_re_relay,n_re_relay,vacant0,vacant,re_symbol,relay_main,has_cov_symbol,T,ima2)
    %反向广播情况
        global density_EM CW_CTB;   
    %我们需要检查反向广播空白区域中的路段标记点是否曾被覆盖过
        recover_index_set = zeros(length(vacant0(:,1)),1);%初始化重复覆盖标记数组为length(vacant0(:,1))行，1列的数组
        %接下来检查vacant0（或可以说是vacant）中的路段标记点是否曾被location_relay中继节点或location_re_relay反向广播中继节点覆盖过，如果覆盖过一次，则recover参数置1表重复覆盖
        for i = 1:length(vacant0(:,1))
            recover = 0;%初始化重复覆盖参数为0           
            for j = 1:length(location_relay(:,1))
                if location_relay(j,1)==0
                    break;
                elseif (sum((location_relay(j,2:3)-vacant0(i,1:2)).^2))^0.5/Linmap1m<R%如果空白区域中的点在location_relay数组中的历代中继节点中的通信范围内，则看其是否被障碍物阻碍过
                    recover = 1;%初始化重复覆盖标记位为1
                    %下面为检测是否被障碍物阻碍代码
                    sample_num = 100;%初始化抽样次数为100
                    det_X = (vacant0(i,1)-location_relay(j,2))/sample_num;
                    det_Y = (vacant0(i,2)-location_relay(j,3))/sample_num;
                    for n = 1:sample_num
                        temp_x = location_relay(j,2)+n*det_X;
                        temp_y = location_relay(j,3)+n*det_Y;
                        x_data = ceil(temp_x);
                        y_data = ceil(temp_y);
                        if ima2(y_data,x_data) == 0    %如果坐标对应图像矩阵的像素值为0，说明此处为障碍物，说明车辆被障碍物阻碍了                            
                            recover = 0;%将重复覆盖标记位置0，表示其实没有重复覆盖
                            break;
                        end
                    end    
                    %检测完毕，如果被阻碍，则重复覆盖标志位置0
                end
            end
            
            for j = 1:length(location_re_relay(:,1))             
                if location_re_relay(j,1)==0
                    break;
                elseif (sum((location_re_relay(j,2:3)-vacant0(i,1:2)).^2))^0.5/Linmap1m<R%如果空白区域中的点在location_re_relay数组中的历代中继节点中的通信范围内，则看其是否被障碍物阻碍过                  
                    recover = 1;%初始化重复覆盖标记位为1
                    %下面为检测是否被障碍物阻碍代码
                    sample_num = 100;%初始化抽样次数为100
                    det_X = (vacant0(i,1)-location_re_relay(j,2))/sample_num;
                    det_Y = (vacant0(i,2)-location_re_relay(j,3))/sample_num;
                    for n = 1:sample_num
                        temp_x = location_re_relay(j,2)+n*det_X;
                        temp_y = location_re_relay(j,3)+n*det_Y;
                        x_data = ceil(temp_x);
                        y_data = ceil(temp_y);
                        if ima2(y_data,x_data) == 0    %如果坐标对应图像矩阵的像素值为0，说明此处为障碍物，说明车辆被障碍物阻碍了                            
                            recover = 0;%将重复覆盖标记位置0，表示其实没有重复覆盖
                            break;
                        end
                    end 
                    %检测完毕，如果被阻碍，则重复覆盖标志位置0
                end
            end           
            recover_index_set(i,1) = recover;%将recover的值存入数组中
        end
        
        if isempty(find(recover_index_set==0, 1))%如果寻找recover_index_set（重复覆盖标记数组）中未被覆盖过的路段标记点返回的结果是空（即recover=0的一个都没有，全是recover=1重复覆盖），即表示vacant0（或vacant）中的所有路段点都曾被覆盖过一次了
            re_symbol=0;%则将反向广播标记置0，不进入反向广播
            has_cov_symbol = 1;%1表示空白区域已经被覆盖过了
        end
    %结束空白区域中的路段点是否曾被覆盖的检查
    
    if re_symbol == 1 %如果反向广播标志位为1，进入反向广播
        r = length(vacant0(:,1));%vacant（或vacant0）数组的有效行数，即非零行数
        r_end = length(vacant(:,1));
        for i = r+1:r_end
            vacant(r+1,:)=[];%清除掉vacant数组中第r行之后的行数
        end
        
        if re_t == 0
            re_t = t;  %如果反向广播的计时re_t为0，表示之前没有进入过反向广播，则初始化反向广播的计时从t开始
        end
        if n_relay>=2            
            vacant(:,3) = 0;
            re_start_road_index = jyc_find_relayroad(locationmark,relay_main);
            if send_start(1,3)>re_start_road_index
                re_send_end(1,1:2) = vacant(vacant(:,4)==max(vacant(:,4)),1:2);   
                re_send_end(1,3) = vacant(vacant(:,4)==max(vacant(:,4)),4); 
            else
                re_send_end(1,1:2) = vacant(vacant(:,4)==min(vacant(:,4)),1:2);
                re_send_end(1,3) = vacant(vacant(:,4)==min(vacant(:,4)),4); 
            end            
        elseif n_relay>2            
            vacant(:,3) = 0;
            if location_relay(n_relay-2,4)>location_relay(n_relay-1,4)
                re_send_end(1,1:2) = vacant(vacant(:,4)==max(vacant(:,4)),1:2); 
                re_send_end(1,3) = vacant(vacant(:,4)==max(vacant(:,4)),4); 
            else
                re_send_end(1,1:2) = vacant(vacant(:,4)==min(vacant(:,4)),1:2); 
                re_send_end(1,3) = vacant(vacant(:,4)==min(vacant(:,4)),4); 
            end
        end    
%         plot(re_send_end(1),re_send_end(2),'o','LineWidth',1,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',6);%标出反向广播的终点位置
%         hold on
        re_send_start = relay_main;        
        re_relay = re_send_start;%re_relay开始值为re_send_start
        re_relayend = re_send_end;%确定反向广播终点位置        
        n_re_relay = 1;
        PDR = [];%初始化PDR为空
        %反向广播与正常的弯道广播方案一致,以下都是反向广播情景
        while isempty(vacant0)==0 &&  re_t <= T
            re_relay_candi = zeros(length(location_vehi(:,1)),3);%初始化re_relay_candi（反向广播中继节点备用节点矩阵）为length(location_vehi(:,1))行，3列的全零矩阵
            sender = re_relay;
            re_relay_location(n_re_relay,1) = n_re_relay;
            re_relay_location(n_re_relay,2:3) = re_relay;
            re_relay_location(n_re_relay,4) = 2;%re_relay_location保存所有反向广播中继节点，1：序号； 2，3：(x,y)； 4：场景(0出现空白区域，1直道，2弯道，3十字路口)
            continu = 1;%初始化是否连续标志位置1，1表连续，0表不连续
            [~,location_vehi_in_Rp]=jyc_bar_posi_opt(location_vehi,locationmark,R,Linmap1m,re_relay,re_send_end,ima2);     %只用了寻找最优位置函数筛选通信范围内车辆节点的功能
            [re_relay_road_index]=jyc_find_relayroad(locationmark,re_relay);                      
            for i = 1:length(vacant0(:,1))-1
                if vacant0(i,3)+1 ~= vacant0(i+1,3)%如果不连续
                    continu = 0;%将连续标志位置0
                    break;
                else%如果连续
                    continu = 1;%将连续标志位置1
                end
            end
            
            for j = 1:length(locationmark(:,1))
                if (sum((re_relay-locationmark(j,1:2)).^2))^0.5/Linmap1m>R && j>re_relay_road_index%找出路段编号上超过re_relay的第一个在re_relay通信范围外的路段标记点
                    if j<=re_send_end(1,3)%如果该路段标记点编号小于等于re_send_end所在路段编号，则可直接选其作为最优位置所在路段
                        for n = re_relay_road_index+1:j-1
                            sample_num = 100;%抽样次数取100，判断是否被障碍物阻碍
                            for m = 1:sample_num
                                detx = (locationmark(n,1)-re_relay(1,1))/sample_num;%求出detx
                                dety = (locationmark(n,2)-re_relay(1,2))/sample_num;%求出dety
                                x_data = ceil(re_relay(1,1)+m*detx);
                                y_data = ceil(re_relay(1,2)+m*dety);%向大取整x_data,y_data的值，准备放入ima2中查看是否被障碍物阻挡
                                if ima2(y_data,x_data) == 0%如果坐标对应图像矩阵的像素值为0，说明此处为障碍物，说明vacant0中的路段标记点被障碍物阻碍了
                                    break;
                                elseif m == sample_num && ima2(y_data,x_data) ~= 0
                                    posi_opt_mark = n;                                   
                                end
                            end
                        end
                        break;
                    else
                        for n = re_relay_road_index+1:re_send_end(1,3)                           
                            sample_num = 100;%抽样次数取100，判断是否被障碍物阻碍
                            for m = 1:sample_num
                                detx = (locationmark(n,1)-re_relay(1,1))/sample_num;%求出detx
                                dety = (locationmark(n,2)-re_relay(1,2))/sample_num;%求出dety
                                x_data = ceil(re_relay(1,1)+m*detx);
                                y_data = ceil(re_relay(1,2)+m*dety);%向大取整x_data,y_data的值，准备放入ima2中查看是否被障碍物阻挡
                                if ima2(y_data,x_data) == 0%如果坐标对应图像矩阵的像素值为0，说明此处为障碍物，说明vacant0中的路段标记点被障碍物阻碍了
                                    break;
                                elseif m == sample_num && ima2(y_data,x_data) ~= 0
                                    posi_opt_mark = n;
                                end
                            end                       
                        end
                        break;
                    end
                end
            end


%             plot(locationmark(posi_opt_mark,1),locationmark(posi_opt_mark,2),'o','LineWidth',1,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',4);
%             hold on
                                    
            sample_num = ceil(((sum((locationmark(posi_opt_mark,1:2)-locationmark(posi_opt_mark+1,1:2)).^2))^0.5/Linmap1m)/10);%每10米抽样，可抽样的次数(向大取整)
            detx = (locationmark(posi_opt_mark+1,1)-locationmark(posi_opt_mark,1))/sample_num;%x递增量
            dety = (locationmark(posi_opt_mark+1,2)-locationmark(posi_opt_mark,2))/sample_num;%y递增量
            for n = 1:sample_num
                posi_opt(1,1) = locationmark(posi_opt_mark,1)+n*detx;
                posi_opt(1,2) = locationmark(posi_opt_mark,2)+n*dety;
                
                cross_bar = [0 0];%初始化障碍物阻碍标记为[0 0],第一个数表示上一个点是否被障碍物阻碍的标志，第二个数表示当前点是否被障碍物阻碍
                sample_num = 100;%抽样次数取100，判断是否被障碍物阻碍
                for m = 1:sample_num
                    detx = (posi_opt(1,1)-re_relay(1,1))/sample_num;%求出detx
                    dety = (posi_opt(1,2)-re_relay(1,2))/sample_num;%求出dety
                    x_data = ceil(re_relay(1,1)+m*detx);
                    y_data = ceil(re_relay(1,2)+m*dety);%向大取整x_data,y_data的值，准备放入ima2中查看是否被障碍物阻挡
                    if ima2(y_data,x_data) == 0%如果坐标对应图像矩阵的像素值为0，说明此处为障碍物，说明vacant0中的路段标记点被障碍物阻碍了
                        cross_bar(1,1) = cross_bar(1,2);
                        cross_bar(1,2) = 1;
                        break;                   
                    end
                end
                if (sum((re_relay-posi_opt).^2))^0.5/Linmap1m>R%如果posi_opt超出通信边界，那么选取上一个posi_opt作为真正的posi_opt
                    posi_opt(1,1) = locationmark(posi_opt_mark,1)+(n-1)*detx;
                    posi_opt(1,2) = locationmark(posi_opt_mark,2)+(n-1)*dety;
                    break;
                else%如果posi_opt没有超出通信边界
                    if cross_bar(1,2)==1 && cross_bar(1,1)==0%如果上一个posi_opt未被障碍物阻碍，而当前的posi_opt被障碍物阻碍，则我们选上一个posi_opt作为真正的posi_opt
                        posi_opt(1,1) = locationmark(posi_opt_mark,1)+(n-1)*detx;
                        posi_opt(1,2) = locationmark(posi_opt_mark,2)+(n-1)*dety;
                        break;
                    end
                end
            end
                    
           % if continu == 1%如果连续
           if isempty(location_vehi_in_Rp)
               re_relay = posi_opt;
           else 
               re_relay_candi_index = 1;
               for j = 1:length(location_vehi_in_Rp(:,1))
                   if re_relay_road_index<location_vehi_in_Rp(j,4) && location_vehi_in_Rp(j,4)<=posi_opt_mark 
                       re_relay_candi(re_relay_candi_index,1:3) = location_vehi_in_Rp(j,2:4);
                       re_relay_candi_index = re_relay_candi_index+1;
                   end
               end
               re_relay_candi(re_relay_candi(:,3)==0,:)=[];%将re_relay_candi全零行删除
               if isempty(re_relay_candi)
                   re_relay = posi_opt;
               else
                   re_relay = re_relay_candi(re_relay_candi(:,3)==max(re_relay_candi(:,3)),1:2);
               end
           end
           if length(re_relay(:,1))>1
               dis_relay_popt = zeros(length(re_relay(:,1)),1);%初始化dis_relay_popt参数用来存放反向relay与Popt点的距离
               for ii = 1:length(re_relay(:,1))
                   dis_relay_popt(ii,1) = (sum((re_relay(ii,1:2)-posi_opt(1,1:2)).^2))^0.5/Linmap1m;                   
               end
               [re_relay_index,~] = find(dis_relay_popt(:,1)==min(dis_relay_popt));
               if length(re_relay_index(:,1))>1
                   re_relay_index_temp = re_relay_index(1,1);
                   re_relay_index = [];
                   re_relay_index = re_relay_index_temp;
               end
               re_relay_temp = re_relay(re_relay_index,:);
               re_relay = [];
               re_relay = re_relay_temp;
           end
%             plot(posi_opt(1),posi_opt(2),'*','LineWidth',1,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',8);
%             hold on
%             plot(re_relay(1,1),re_relay(1,2),'v','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',6);
%             hold on
            
            %更新vacant0的内容
            for j = 1:length(vacant0(:,1))
                crossbar = 0;%穿过障碍物标志位初始化置0，0表没有穿过，1表示穿过，穿过障碍物意思是路段标记点与re_relay的连线穿过障碍物，表示路段标记点被障碍物阻碍了
                if (sum((re_relay-vacant0(j,1:2)).^2))^0.5/Linmap1m<=R
                    sample_num = ceil(((sum((re_relay-vacant0(j,1:2)).^2))^0.5/Linmap1m)/10);%求出10m递进的抽样次数,向大取整
                    for n = 1:sample_num
                        detx = (vacant0(j,1)-re_relay(1,1))/sample_num;%求出detx
                        dety = (vacant0(j,2)-re_relay(1,2))/sample_num;%求出dety
                        x_data = ceil(re_relay(1,1)+n*detx);
                        y_data = ceil(re_relay(1,2)+n*dety);%向大取整x_data,y_data的值，准备放入ima2中查看是否被障碍物阻挡
                        if ima2(y_data,x_data) == 0%如果坐标对应图像矩阵的像素值为0，说明此处为障碍物，说明vacant0中的路段标记点被障碍物阻碍了
                            crossbar = 1;%穿过障碍物标志位置1
                            break;
                        end
                    end
                    if crossbar == 0%如果穿过障碍物标志位置0，说明路段标记点没有被障碍物阻碍
                        vacant0(j,:) = 0;
                    end
                end                                 
            end
            vacant0(vacant0(:,3)==0,:) = [];%清除掉vacant0中的全零行，表示完成vacant0的更新    
            [delay_one_hop_average,PDR_part,re_relay] = jyc_3P3B_delay_calcu (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,re_relay);
            re_t = re_t+delay_one_hop_average;
            PDR = [PDR;PDR_part];%while循环中无限加PDR_part至PDR数组中
            
            %更新vacant0的内容完毕       
            location_re_relay(n_re_relay,1) = n_re_relay;
            location_re_relay(n_re_relay,2:3) = re_relay;
            location_re_relay(n_re_relay,4) = 2;%location_re_relay(n_re_relay,4) = 2 表示弯道场景下的中继节点选择
            n_re_relay = n_re_relay+1;

        end
        if (sum((re_relay(1:2)-re_relayend(1:2)).^2))^0.5/Linmap1m <= R
            re_relay = re_relayend;
%             plot(re_relay(1),re_relay(2),'v','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',6);
%             hold on
            location_re_relay(n_re_relay,1) = n_re_relay;
            location_re_relay(n_re_relay,2:3) = re_relay(1:2);
            location_re_relay(n_re_relay,4) = 2;%location_re_relay(n_re_relay,4) = 2 表示弯道场景下的中继节点选择
            n_re_relay = n_re_relay+1;
            has_cov_symbol = 1;%放在反向广播的while后，表示已经将空白区域覆盖
           % re_relay_main = re_relay;
        end 
    end
    last_t = delay_one_hop_average;
end