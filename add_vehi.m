function [location_vehi,num_vehi,relay]=add_vehi(locationmark,relay,Linmap1m,ima2,posi_opt,send_end,R,num_vehi,location_vehi,old_sender,old_posi_opt)
%������ֻ����relay����·�α�Ŵ���send_end������û�з���㲥������relay��junction���R/2����֮������������û������whileѭ����
global locationjunction;
isjunction=0;%posi_opt�Ƿ�junction���жϣ�0��ʾ���ǣ�1��ʾ��
suc = 0;%�ɹ���־����ʼ��Ϊ0
for i = 1:length(locationjunction(:,1))
    if posi_opt(1) == locationjunction(i,2) && posi_opt(2) == locationjunction(i,3)
        isjunction=1;
        break; 
    end
end
[relay_road_index]=jyc_find_relayroad(locationmark,relay);%���relay����·�α��
[posi_opt_road_index]=jyc_find_relayroad(locationmark,posi_opt);%���posi_opt����·�α��
dis_relay_Popt = (sum((relay-posi_opt).^2))^0.5/Linmap1m;
if dis_relay_Popt>R
    dis_relay_Popt = R;
end

while (suc==0)
    if isjunction==0 %���posi_opt����Junction��
        if dis_relay_Popt<((3/4)*R)%posi_opt��relay�ľ���ԶС��R
            relay = posi_opt;%��ôֱ��ѡ�õ㼴��
            location_vehi(num_vehi+1,1) = num_vehi+1;%��������ӽ�location_vehi������
            location_vehi(num_vehi+1,2:3) = posi_opt;
            location_vehi(num_vehi+1,4) = 0;
            num_vehi = num_vehi+1;
            break;%ֱ������whileѭ��
        else%���posi_opt��relay�ľ���ӽ���R����ôΪ��ֱֹ��ѡPopt�㵼�»����ű���㷨PDRΪ0�Ĵ������࣬������Ҫ�������һ�������ڵ�
            if isempty(old_sender)==0%���old_sender��Ϊ�գ��򱾴�sender���ֳ�����relay����ͨ�ŷ�Χ��������ɵĳ�����������һ��sender���ֳ���old_sender����ͨ�ŷ�Χ��
                dis_old_relay_old_sender = (sum((relay-old_sender).^2))^0.5/Linmap1m;%�����relay���ֳ�����sender������sender֮��ľ���
                %�������������sender��ͨ�ŷ�Χ
                dis_old_Popt_old_sender = (sum((old_posi_opt-old_sender).^2))^0.5/Linmap1m;
                if dis_old_Popt_old_sender>R
                    dis_old_Popt_old_sender = R;
                end
                %��sender��ͨ�ŷ�Χ�����������һ�д������old_sender�Ĺ�ʣ���Ƿ�Χ
                det_dis = dis_old_Popt_old_sender-dis_old_relay_old_sender;%���old_sender�Ĺ�ʣ���Ƿ�Χ
                if det_dis<0%�ų�һЩ΢С�Ĵ������
                    det_dis = 0;
                end
                if dis_relay_Popt-det_dis<1 %���old_sender�Ĺ�ʣ���Ƿ�Χ�뵱ǰsender������relay���ĸ��Ƿ�Χ���С��1����ôֱ��ѡ��ǰsender������λ�����ɳ����ڵ���Ϊ�µ�relay��
                    dis_new_vehi = dis_relay_Popt;   %ֱ��ѡ����λ�þ������ɳ����ڵ�
                else
                    dis_new_vehi = dis_relay_Popt-unidrnd(fix((dis_relay_Popt-det_dis)));   %��det_dis��dis_relay_Popt֮�����һ�����룬����������ĳ���������old_sender��ͨ�ŷ�Χ�ڣ���Ȼ�Ļ���old_sender�Ϳ�������ͨ�ţ�Ϊ��Ҫ�����ڲ�����ͨ�ţ��ⲻ����
                end
            else
                dis_new_vehi = unidrnd(fix(dis_relay_Popt));   %��1��dis_relay_Popt֮�����һ������
            end
        end
    else%���posi_opt��junction�㣬˵��������junction��ľ�������R/2��R֮�䣬��Ϊ��junction��ľ�����R/2���ڵ�������ܽ��뱾����
        dis_new_vehi = dis_relay_Popt-unidrnd(fix(dis_relay_Popt/2));%���һ����dis_relay_Popt/2��dis_relay_Popt֮��Ľϴ�ľ��룬��Ŀ����Ϊ�˱�֤�ڱ���relayѡ���󣬾��Կ��������м̽ڵ�ѡ������whileѭ�����Է�ֹ�ڴ�������������������Ϊ�������relay�����С����ô��������������֮ǰ��ֱ��ѡ�б���С����relay������ȴû��ѡ����Ϊ��ʱ��С����relay��δ���ɣ�����ʵ��������������ϳ���
    end
    if relay_road_index>send_end(3)           %relay����·�α�Ŵ���send_end����·�α��
        for i = 1:length(locationmark(:,1))
            if (sum((locationmark(i,1:2)-relay).^2))^0.5/Linmap1m<=dis_new_vehi && relay_road_index>i && i>=posi_opt_road_index%��·�α���������ҳ���relay�ľ���С�ڵ���dis_new_vehi����send_end��ʼ��relay���򣬵�һ��������locationmark�㣩���ұ��С��relay_road_index���ڵ���posi_opt_road_index��·�α�ǵ�
                if i>1
                    %��ʼ��i·�γ������ҳ���·���Ͼ�relay����Ϊdis_new_vehi���ҵĵ�temprelay
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
                %��ʼ�����Ƿ��ϰ����谭�ж�
                sample_for_bar = 100;%��ʼ����������Ϊ100
                det_X = (temprelay(1)-relay(1))/sample_for_bar;
                det_Y = (temprelay(2)-relay(2))/sample_for_bar;
                for j = 1:sample_for_bar
                    cross_bar = 0;%�������ϰ���ı��λ��ʼ��Ϊ0
                    temp_x = relay(1)+j*det_X;
                    temp_y = relay(2)+j*det_Y;
                    x_data = ceil(temp_x);
                    y_data = ceil(temp_y);
                    if ima2(y_data,x_data) == 0    %��������Ӧͼ����������ֵΪ0��˵��˳�����ߵݽ�ʱ�������ϰ��˵�����ߴ������ϰ���
                        cross_bar = 1;    %�������ϰ���ı��λ��1
                        break;
                    end
                end
                %�Ƿ��ϰ����谭�ж����
                if (sum((temprelay-relay).^2))^0.5/Linmap1m<R && cross_bar == 0%��������ҵ���·�α�ǵ��relay�ľ���С��R����û�б��ϰ����谭
                    relay = temprelay;%�����ǽ�����Ϊ���ǵ�relay��
                    location_vehi(num_vehi+1,1) = num_vehi+1;%��������ӽ�location_vehi������
                    location_vehi(num_vehi+1,2:3) = temprelay;
                    location_vehi(num_vehi+1,4) = 0;
                    num_vehi = num_vehi+1;
                    suc = 1;%�ɹ���־��1
                    break;
                end
            end
        end
    end
    
end