function [locationmark_in_relay,ang_max,ang_min]=mark_inrelay_nohinder(relay,locationbar,Linmap1m,locationmark,R)
    %���������ϰ������relay�㲥ʱ�����ҽǶ�ƫ�����ĺ�����ÿ���ϰ���Ϊ���������ĸ��ϰ����ǵ㹹�ɣ����ǵ���Ŵ�ͼ�е���ൽ�Ҳ����ε����������Ϊ��һ�ϰ���
    %bar_num = length(locationbar)/4;%����ϰ���ĸ���Ϊbar_num������Ϊ�ϰ����ǳ����Σ����ĸ��ϰ����ǵ㹹�ɣ��ϰ����ǵ㶼����locationbar������
    %bar_around_relay = zeros(2,1);%��ʼ��bar_around_relay����,��������������Ϊrelay������������ϰ������

        [relay_road_index]=jyc_find_relayroad(locationmark,relay);%���relay����·��
        
        [relay_arc,~]= cart2pol(locationmark(relay_road_index,2)-relay(2),locationmark(relay_road_index,1)-relay(1));%���relay����·�εĶ�Ӧ·�ε���relay�����ߵĻ���
        relay_ang = relay_arc*180/pi;%���relay����·�νǶȡ�    �Ƕ�Ӧ�ð������м̽ڵ�ѡ��ķ������ǲ�������������

        relay_locationbar_dis = zeros(length(locationbar(:,1)),2);%��ʼ��relay_locationbar_dis���飬��������м̽ڵ����ϰ����ǵ�ľ��룬��һ�д���ţ��ڶ��д����
        for i = 1:length(locationbar(:,1))
            relay_locationbar_dis(i,1)=i;%��һ�����������
            relay_locationbar_dis(i,2)=(sum((relay(1:2)-locationbar(i,1:2)).^2))^0.5/Linmap1m;%�ڶ�����������룬����ϰ������ǵ���relay�ľ��룬��Ӧλ�õش���relay_locationbar_dis������
        end
        
        relay_locationbar_dis((relay_locationbar_dis(:,2)>R),:)=[];%ɾ����ͨ�ŷ�ΧR����ϰ����ǵ�
        ang_bar_relay = zeros(length(relay_locationbar_dis(:,1)),3);%��ʼ�������ϰ�����м̽ڵ����߽Ƕȵ�����ang_bar_relay
        for i = 1:length(relay_locationbar_dis(:,1))
            ang_bar_relay(i,1) = relay_locationbar_dis(i,1);
            [relay_bar_arc,~]= cart2pol(locationbar(relay_locationbar_dis(i,1),2)-relay(2),locationbar(relay_locationbar_dis(i,1),1)-relay(1));
            ang_bar_relay(i,2) = relay_bar_arc*180/pi;
            ang_bar_relay(i,3) = ceil(ang_bar_relay(i,1)/4);
        end
        %��������·���
        locationmark_in_relay = zeros(length(locationmark(:,1)),3);%��ʼ�������м̽ڵ�ͨ�ŷ�Χ�ڵ�·�α�ǵ�����飬��һ�д�·�α�ǵ��ţ��ڶ��д�·�α�ǵ��Xֵ�������д�·�α�ǵ��Yֵ
        j = 1;
        for i = 1:length(locationmark(:,1))
            if (sum((locationmark(i,1:2)-relay(1:2)).^2))^0.5/Linmap1m < R
                locationmark_in_relay(j,1) = i;
                locationmark_in_relay(j,2:3) = locationmark(i,1:2);
                j = j+1;
            end
        end
        locationmark_in_relay(locationmark_in_relay(:,1)==0,:)=[];
        ang_min = zeros(ang_bar_relay(length(ang_bar_relay(:,1)),3),4);%��ʼ��ang_min���飬����Ϊ�ϰ�����������Ϊ4����һ��Ϊ�ϰ����ǵ������ϰ�����ţ��ڶ���Ϊ�ϰ����ǵ���relay����С�Ƕ�ֵ��������Ϊ�ϰ����ǵ��Xֵ��������ΪYֵ
        ang_max = zeros(ang_bar_relay(length(ang_bar_relay(:,1)),3),4);%��ʼ��ang_max���飬����Ϊ�ϰ�����������Ϊ4����һ��Ϊ�ϰ����ǵ������ϰ�����ţ��ڶ���Ϊ�ϰ����ǵ���relay�����Ƕ�ֵ��������Ϊ�ϰ����ǵ��Xֵ��������ΪYֵ
       %������Щ�����Ǻ�ӵ�
        ang_bar_relay_dev = zeros(ang_bar_relay(length(ang_bar_relay(:,1)),3),4);%��ʼ��ang_bar_relay_dev���飬��һ��Ϊ�ϰ����ǵ���ţ��ڶ���Ϊ����X�ᣨ����X�������ᣩƫ�ƽǶȵ�ֵ��������Ϊ�����ϰ����ţ�������Ϊbar_relay��������ֱ������ϵ������
        temp_ang_max = zeros(ang_bar_relay(length(ang_bar_relay(:,1)),3),5);%ͬang_max��ֻ�����ڶ�����ƫ�ƽ�ֵ��������������ֱ������ϵ����ֵ
        temp_ang_min = zeros(ang_bar_relay(length(ang_bar_relay(:,1)),3),5);%ͬang_min��ֻ�����ڶ�����ƫ�ƽ�ֵ��������������ֱ������ϵ����ֵ
        for j = 1:length(ang_bar_relay(:,1))
             %���������Ǻ�ӵ�
            ang_bar_relay_dev(j,1) = ang_bar_relay(j,1);
            ang_bar_relay_dev(j,3) = ang_bar_relay(j,3);
            if ang_bar_relay(j,2) >= 0 && ang_bar_relay(j,2) < 90
                ang_bar_relay_dev(j,2) = ang_bar_relay(j,2)-0;
                ang_bar_relay_dev(j,4) = 1;%λ�ڵ�һ����
            elseif ang_bar_relay(j,2) >=90 && ang_bar_relay(j,2) <= 180
                ang_bar_relay_dev(j,2) = 180-ang_bar_relay(j,2);
                ang_bar_relay_dev(j,4) = 2;%λ�ڵڶ�����
            elseif ang_bar_relay(j,2) < 0 && ang_bar_relay(j,2) > -90
                ang_bar_relay_dev(j,2) = 0-ang_bar_relay(j,2);
                ang_bar_relay_dev(j,4) = 4;%λ�ڵ�������
            elseif ang_bar_relay(j,2) <= -90 && ang_bar_relay(j,2) >= -180
                ang_bar_relay_dev(j,2) = ang_bar_relay(j,2)-(-180);
                ang_bar_relay_dev(j,4) = 3;%λ�ڵ�������
            end                        
            %������ݵ���Ϊֹ��Ŀ��Ϊ�˸��õ�ɸѡ���ϰ����谭�ĽǶȷ�Χ,��forѭ����֤�������ȷ�� 
        end
        sign = 0;
        for j = 1:length(ang_bar_relay(:,1))           
            if temp_ang_max(ang_bar_relay(j,3),1)~= ang_bar_relay(j,3)%����Ѿ�������һ���ϰ�����                
                temp_ang_max(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                temp_ang_max(ang_bar_relay(j,3),2) = ang_bar_relay_dev(j,2);
                temp_ang_max(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                temp_ang_max(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);
                sign = 0;
                [rank,~] = find(ang_bar_relay_dev(:,4)~=temp_ang_max(ang_bar_relay(j,3),5) & ang_bar_relay_dev(:,3)==temp_ang_max(ang_bar_relay(j,3),1));%��ang_bar_relay_dev������ɸѡ����temp_ang_max��ang_bar_relay(j,3)��ͬ��һ���ϰ���ȴ���ڲ�ͬ������ϰ����ǵ�
                if isempty(rank)
                    temp_ang_min(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                    temp_ang_min(ang_bar_relay(j,3),2) = 0;
                    temp_ang_min(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                    temp_ang_min(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);
                    sign = 1;%��1��ʾ���ϰ������嶼��ĳһ��������
                else
                    temp_ang_min(ang_bar_relay(j,3),1) = ang_bar_relay(rank(1),3);
                    temp_ang_min(ang_bar_relay(j,3),2) = ang_bar_relay_dev(rank(1),2);
                    temp_ang_min(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(rank(1),1),1:2);
                    temp_ang_min(ang_bar_relay(j,3),5) = ang_bar_relay_dev(rank(1),4);
                end
                %�����������ϰ�����������������  
            elseif temp_ang_max(ang_bar_relay(j,3),5) == ang_bar_relay_dev(j,4) && sign==0%��temp_ang_max��˵���������һ�������ϣ��ұ��Ϊ0�����Ϊ0��ʾ�������ϰ������������䣩
                if temp_ang_max(ang_bar_relay(j,3),2) > ang_bar_relay_dev(j,2)%���temp_ang_max�Ǹ�����ƫ��X������
                    temp_ang_max(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                    temp_ang_max(ang_bar_relay(j,3),2) = ang_bar_relay_dev(j,2);
                    temp_ang_max(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                    temp_ang_max(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);%ѡ��ƫ��X�����ٵķ��� temp_ang_max ������
                end
            elseif temp_ang_min(ang_bar_relay(j,3),5) == ang_bar_relay_dev(j,4) && sign==0%��temp_ang_min��˵���������һ�������ϣ��ұ��Ϊ0
                if temp_ang_min(ang_bar_relay(j,3),2) > ang_bar_relay_dev(j,2)%���temp_ang_min�Ǹ�����ƫ��X������
                    temp_ang_min(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                    temp_ang_min(ang_bar_relay(j,3),2) = ang_bar_relay_dev(j,2);
                    temp_ang_min(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                    temp_ang_min(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);%%ѡ��ƫ��X�����ٵķ��� temp_ang_min ������
                end
                %�����������ϰ��ﶼ��һ�������ڵ����
            elseif temp_ang_max(ang_bar_relay(j,3),5) == ang_bar_relay_dev(j,4) && sign==1%��temp_ang_max��˵�������һ�����䣬�ұ��Ϊ1�����Ϊ1��ʾ�����ϰ��ﶼ��һ�������ڣ�
                if temp_ang_max(ang_bar_relay(j,3),2) < ang_bar_relay_dev(j,2)%���temp_ang_max���Ǹ�����ƫ��X������
                    temp_ang_max(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                    temp_ang_max(ang_bar_relay(j,3),2) = ang_bar_relay_dev(j,2);
                    temp_ang_max(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                    temp_ang_max(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);%ѡ��ƫ��X�����ķ��� temp_ang_max ������
                end
            elseif temp_ang_min(ang_bar_relay(j,3),5) == ang_bar_relay_dev(j,4) && sign==1%��temp_ang_min��˵�������һ�����䣬�ұ��Ϊ1
                if temp_ang_min(ang_bar_relay(j,3),2) > ang_bar_relay_dev(j,2)%���temp_ang_min�Ǹ�����ƫ��X������
                    temp_ang_min(ang_bar_relay(j,3),1) = ang_bar_relay(j,3);
                    temp_ang_min(ang_bar_relay(j,3),2) = ang_bar_relay_dev(j,2);
                    temp_ang_min(ang_bar_relay(j,3),3:4) = locationbar(ang_bar_relay(j,1),1:2);
                    temp_ang_min(ang_bar_relay(j,3),5) = ang_bar_relay_dev(j,4);%%ѡ��ƫ��X�����ٵķ��� temp_ang_min ������
                end
            end%if��䵽�˽���
        end
        for j = 1:length(temp_ang_max(:,1))
            %���¿�ʼ�жϸ�ֵ����
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
                %����Max��Min����������жϸ�ֵ����
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
                end %��������жϸ�ֵ���ڽ���   
                                                        
            end%���жϸ�ֵ���ڽ���
                   
        end%������֤�������ȷ��
        
        relay_mark_arc = 0;%��ʼ���м̽ڵ��·�α�ǵ����߽ǶȲ���Ϊ0
        relay_mark_ang = 0;%��ʼ���м̽ڵ��·�α�ǵ����߽ǶȲ���Ϊ0��
        ang_max(ang_max(:,1)==0,:)=[];%ɾ��ang_max�����е�ȫ����
        ang_min(ang_min(:,1)==0,:)=[];%ɾ��ang_min�����е�ȫ����
        if length(ang_max(:,1)) > length(ang_min(:,1))
           ang_max(length(ang_max(:,1)),:)=[];
        elseif length(ang_max(:,1)) < length(ang_min(:,1))
           ang_min(length(ang_min(:,1)),:)=[]; 
        end
        for i = 1:length(locationmark_in_relay(:,1))
            [relay_mark_arc,~]= cart2pol(locationmark_in_relay(i,3)-relay(2),locationmark_in_relay(i,2)-relay(1));%locationmark_in_relay�����е�1��Ϊ·�α�ǵ��ţ���2��ΪX����3��ΪY
            dis_mark_relay = (sum((locationmark_in_relay(i,2:3)-relay(1:2)).^2))^0.5/Linmap1m;
            relay_mark_ang = relay_mark_arc*180/pi;
            for j = 1:length(ang_max(:,1))
                tempmin = zeros(1,3);%��ʱMin��ά�����ʾ������ά��0
                tempmax = zeros(1,3);%ͬ��
                temprelay = zeros(1,3);%ͬ��
                if ang_max(j,2)>ang_min(j,2)
                    if relay_mark_ang<ang_max(j,2) && relay_mark_ang>ang_min(j,2)%�����������˵��·�ε��п��ܱ��ϰ����赲������������·�ε������ϰ����relay֮�䣬�������ϰ������
                        tempmin(1,1:2) = ang_min(j,3:4);%��1��2ά��ӦX��Y����
                        tempmin(1,3) = 0;%����ά��0
                        tempmax(1,1:2) = ang_max(j,3:4);
                        tempmax(1,3) = 0;
                        temprelay(1,1:2) = relay(1:2);
                        temprelay(1,3) = 0;
                        dis_relay_barline = (norm(cross(tempmin-tempmax,temprelay-tempmax))/norm(tempmin-tempmax))/Linmap1m;
                        if dis_relay_barline < dis_mark_relay %relay���ϰ������֮��ľ���С��·�α�ǵ���relay�ľ��룬��·�ε������ϰ�����棬����ס��
                            locationmark_in_relay(i,:) = 0;%��relay��Χ�ڵ�·�α�ǵ������и���Ԫ��ȫ����0��������·�ε�����ѶϢ��Ϊ0�������н��ں��汻�����
                        end
                    end
                else
                    if (relay_mark_ang>=0 && relay_mark_ang<ang_max(j,2)) || (relay_mark_ang>ang_min(j,2) && relay_mark_ang<360)%�ر��жϵ���һ�����������
                        tempmin(1,1:2) = ang_min(j,3:4);%��1��2ά��ӦX��Y����
                        tempmin(1,3) = 0;%����ά��0
                        tempmax(1,1:2) = ang_max(j,3:4);
                        tempmax(1,3) = 0;
                        temprelay(1,1:2) = relay(1:2);
                        temprelay(1,3) = 0;
                        dis_relay_barline = (norm(cross(tempmin-tempmax,temprelay-tempmax))/norm(tempmin-tempmax))/Linmap1m;
                        if dis_relay_barline < dis_mark_relay %relay���ϰ������֮��ľ���С��·�α�ǵ���relay�ľ��룬��·�ε������ϰ�����棬����ס��
                            locationmark_in_relay(i,:) = 0;%��relay��Χ�ڵ�·�α�ǵ������и���Ԫ��ȫ����0��������·�ε�����ѶϢ��Ϊ0�������н��ں��汻�����
                        end                         
                    end
                end
            end
        end
        locationmark_in_relay(locationmark_in_relay(:,1)==0,:)=[];%�����locationmark_in_relay�����е�����
        %�·�������
end
    
  