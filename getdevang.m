function [left_dev_ang,right_dev_ang,facebar_sign]=getdevang(relay,locationbar,Linmap1m,locationmark,send_end,R)
    %���������ϰ������relay�㲥ʱ�����ҽǶ�ƫ�����ĺ�����ÿ���ϰ���Ϊ���������ĸ��ϰ����ǵ㹹�ɣ����ǵ���Ŵ�ͼ�е���ൽ�Ҳ����ε����������Ϊ��һ�ϰ���
    %bar_num = length(locationbar)/4;%����ϰ���ĸ���Ϊbar_num������Ϊ�ϰ����ǳ����Σ����ĸ��ϰ����ǵ㹹�ɣ��ϰ����ǵ㶼����locationbar������
    %bar_around_relay = zeros(2,1);%��ʼ��bar_around_relay����,��������������Ϊrelay������������ϰ������
    facebar_sign = 0;%��ʼ��ӭ���ϰ���������Ϊ0
    left_ang_bar_relay = [];%�ȳ�ʼ��left_ang_bar_relay��right_ang_bar_relay����Ϊ��
    right_ang_bar_relay = [];%֮������������������Ӷ���
    [relay_road_index]=jyc_find_relayroad(locationmark,relay);%���relay����·��
    [relay_arc,~]= cart2pol(locationmark(relay_road_index,2)-relay(2),locationmark(relay_road_index,1)-relay(1));%���relay����·�εĶ�Ӧ·�ε���relay�����ߵĻ���
    %K = tan(relay_arc);%���relay����·��Kֵ��б��ֵ��
    relay_ang = relay_arc*180/pi;%���relay����·�νǶȡ�    �Ƕ�Ӧ�ð������м̽ڵ�ѡ��ķ������ǲ�������������
    % if length(send_end)==2
    relay_locationbar_dis = zeros(length(locationbar(:,1)),2);%��ʼ��relay_locationbar_dis���飬��������м̽ڵ����ϰ����ǵ�ľ��룬��һ�д���ţ��ڶ��д����
    for i = 1:length(locationbar(:,1))
        relay_locationbar_dis(i,1)=i;%��һ�����������
        relay_locationbar_dis(i,2)=(sum((relay(1:2)-locationbar(i,1:2)).^2))^0.5/Linmap1m;%�ڶ�����������룬����ϰ������ǵ���relay�ľ��룬��Ӧλ�õش���relay_locationbar_dis������
    end
    
    relay_locationbar_dis((relay_locationbar_dis(:,2)>R),:)=[];%ɾ����ͨ�ŷ�ΧR����ϰ����ǵ�
    %�����������ǵ���relay����ֱ�ߵĽǶȣ�Ȼ��ȡ�Ƕ���relay����·�νǶȡ��������ֵ��С���ϰ����ǵ�Ϊ���ǿ��ǵ��ϰ����
    %K_bar_relay = zeros(length(relay_locationbar_dis(:,1)),2);%��ʼ�������ϰ�����м̽ڵ�����б�ʵ�����K_bar_relay����һ�д��Ӧlocationbar�е���ţ��ڶ��д�Kֵ
    ang_bar_relay = zeros(length(relay_locationbar_dis(:,1)),3);%��ʼ�������ϰ�����м̽ڵ����߽Ƕȵ�����ang_bar_relay
    for i = 1:length(relay_locationbar_dis(:,1))
        ang_bar_relay(i,1) = relay_locationbar_dis(i,1);
        [relay_bar_arc,~]= cart2pol(locationbar(relay_locationbar_dis(i,1),2)-relay(2),locationbar(relay_locationbar_dis(i,1),1)-relay(1));
        ang_bar_relay(i,2) = relay_bar_arc*180/pi;
        ang_bar_relay(i,3) = ceil(ang_bar_relay(i,1)/4);
    end
    
    %���������ж�relay�Ƿ������ӭ����һ���ϰ��ﵲס�����
    [min_index,~] = find(relay_locationbar_dis==min(relay_locationbar_dis(:,2)));
    min_dis(1:2) = relay_locationbar_dis(min_index,1:2);%�ҵ�������ϰ����ǵ㣬��ô�����ǵ������ϰ�����Ȼ�Ǿ�relay������ϰ���
    bar_name = ceil(min_dis(1)/4);%���������ϰ���ı�ţ��ϰ����Ŵ������ҷֱ���1,2,3,4,5...���ϰ���
    j = 1;
    for i = 1:length(ang_bar_relay(:,1))
        if ceil(ang_bar_relay(i,1)/4) == bar_name
            temp_angvalue(j,1:2) = ang_bar_relay(i,1:2);
            j = j+1;
        end
    end
    if isempty(find(temp_angvalue(:,2)>relay_ang, 1))~=1 && isempty(find(temp_angvalue(:,2)<relay_ang, 1))~=1%���temp_angvalue�мȴ��ڴ���relay_ang�ĽǶ��ִ���С��relay_ang�ĽǶȣ���ô˵��relay����·��ֱ��ֱ�Ӵ����˸��ϰ���
        %������������if��䣬˵��relayӭ����һ���ϰ��ﵲס�����㲥��Ϣ
        max_ang = max(temp_angvalue(:,2));
        min_ang = min(temp_angvalue(:,2));%����ϰ������relay���谭��Χ�Ƕ��Ǵ�min_ang��max_ang
        left_dev_ang = max_ang-relay_ang;%����ƫ�ƶ���
        right_dev_ang = abs(min_ang-relay_ang);%����ƫ�ƶ���
        facebar_sign = 1;%ӭ���ϰ�������1���������������relayͨ��ʱ�ų�relay����ֱ������ƫ��left_dev_ang�㣬����ƫ��right_dev_ang��ķ�Χ
        
    else        
        left_index = 1;
        right_index = 1;
        for i = 1:length(ang_bar_relay(:,1))
            if ang_bar_relay(i,2)-relay_ang>0%���ͨ�ŷ�Χ�ڸ����ϰ����ǵ���relay����ֱ�ߵĽǶȼ�ȥrelay����·�εĽǶȺ�ĽǶ�ƫ���������ƫ��������0�����������ƫ��������
                left_ang_bar_relay(left_index,1) = ang_bar_relay(i,1);
                left_ang_bar_relay(left_index,2) = ang_bar_relay(i,2)-relay_ang;%����Ƕ�ƫ������������������
                left_index = left_index+1;
            else %���ͨ�ŷ�Χ�ڸ����ϰ����ǵ���relay����ֱ�ߵĽǶȼ�ȥrelay����·�εĽǶȺ�ĽǶ�ƫ���������ƫ����С��0�����������ƫ��������
                right_ang_bar_relay(right_index,1) = ang_bar_relay(i,1);
                right_ang_bar_relay(right_index,2) = abs(ang_bar_relay(i,2)-relay_ang);%����Ƕ�ƫ������������������
                right_index = right_index+1;
            end
        end
        if isempty(left_ang_bar_relay)
            left_dev_ang = 360-max(right_ang_bar_relay(:,2));
        else
            left_dev_ang = min(left_ang_bar_relay(:,2));%������ƫ����������ȡ����С��ƫ�ƽǶ���Ϊ����ƫ�ƽǶ�
        end
        if isempty(right_ang_bar_relay)
            right_dev_ang = 360-max(left_ang_bar_relay(:,2));
        else
            right_dev_ang = min(right_ang_bar_relay(:,2));%������ƫ����������ȡ����С��ƫ�ƽǶ���Ϊ����ƫ�ƽǶ�
        end
        
     %   if (relay_ang+left_dev_ang > min(ang_bar_relay(:,2)) && relay_ang+left_dev_ang < max(ang_bar_relay(:,2))) || (relay_ang-right_dev_ang > min(ang_bar_relay(:,2)) && relay_ang-right_dev_ang < max(ang_bar_relay(:,2)))
            
            
     %  end
        
    end
    
end