function  [location_vehi_in_Rp]=jyc_vehi_near_junction_in_Rp_choose(location_vehi,locationmark,R,Linmap1m,relay,send_end,ima2)
        location_vehi_in_Rp = zeros(length(location_vehi(:,1)),4);%��ʼ��location_vehi_in_Rp����
        [relay_road_index]=jyc_find_relayroad(locationmark,relay);
 %ɸѡ����ͨ�ŷ�Χ�ڵ��м̽ڵ�          
        for i = 1:length(location_vehi(:,1))
            sample_num = 100;%��ʼ����������Ϊ100
            det_X = (location_vehi(i,2)-relay(1))/sample_num;
            det_Y = (location_vehi(i,3)-relay(2))/sample_num;
            if (sum((location_vehi(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m > R  %��ͨ�ŷ�Χ�⣬����һ����������Ϣȫ����0
                location_vehi(i,:) = 0;
            else
                for j = 1:sample_num
                    temp_x = relay(1)+j*det_X;
                    temp_y = relay(2)+j*det_Y;
                 %   plot(temp_x,temp_y,'o');
                 %   hold on
                    x_data = ceil(temp_x);
                    y_data = ceil(temp_y);
                    if ima2(y_data,x_data) == 0    %��������Ӧͼ����������ֵΪ0��˵���˴�Ϊ�ϰ��˵���������ϰ����谭��
                        location_vehi(i,:) = 0;    %����һ����������Ϣȫ����0
                        break;
                    end
                end
            end
        end
        location_vehi_in_Rp = location_vehi;
        location_vehi_in_Rp(location_vehi_in_Rp(:,1)==0,:) = [];%��location_vehi_in_Rp������ȫ����ȥ��   
        
        %�������forѭ��Ŀ���ǣ���ÿһ�������ڵ�����·�ε�����������location_vehi_in_Rp����ĵ�����
        for i = 1:length(location_vehi_in_Rp(:,1))   
            [vehi_road_index]=jyc_find_relayroad_near_junction(locationmark,location_vehi_in_Rp(i,2:3),R,Linmap1m);
            location_vehi_in_Rp(i,4) = vehi_road_index;
        end
        
        %���¶�Ŀ����Ϊ�˷�ֹ�м̽ڵ��ѡ����ֵ��˻���һ����Ȼ����ǰ����������һ��������ѭ��
        if relay_road_index>send_end(3)
            for i = 1:length(location_vehi_in_Rp(:,1))
                if location_vehi_in_Rp(i,4)~=-1 && location_vehi_in_Rp(i,4)>=relay_road_index  %�ô��ڵ�����Ϊ�˽����ܳ��ֵ���ͬһ·���Ϸ�������ѡ���������,location_vehi_in_Rp(i,4)==-1�ĵ��ǳ����˱���·junction�ĵ㣬Ҫ����·�ڳ����м̽ڵ�ѡ��
                   location_vehi_in_Rp(i,:) = 0;
                end
            end
        elseif relay_road_index<send_end(3)
            for i = 1:length(location_vehi_in_Rp(:,1))
                if location_vehi_in_Rp(i,4)~=-1 && location_vehi_in_Rp(i,4)<=relay_road_index  %��С�ڵ�����Ϊ�˽����ܳ��ֵ���ͬһ·���Ϸ�������ѡ���������
                   location_vehi_in_Rp(i,:) = 0;                    
                end
            end            
        end
        %�����˴�ʩ���˽���
        location_vehi_in_Rp(location_vehi_in_Rp(:,1)==0,:) = [];%��location_vehi_in_Rp������ȫ����ȥ�� 


end