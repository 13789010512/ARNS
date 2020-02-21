    function  [posi_opt,location_vehi_in_Rp]=jyc_bar_posi_opt(location_vehi,locationmark,R,Linmap1m,relay_main,send_end,ima2)
        %�ô�������ѡ������㷨��ֱ���㷨��Popt�㣬���������send_end�������㲥�¶��Ƕ������еģ�������㷨�еķ���㲥ʱ�����re_send_end�Ƕ������е�
        %��ʱ����ͨ������send_end�Ƿ��е����д������ж��ǽ��з���㲥��Poptѡȡ���������㲥��Poptѡȡ
        %road_location_max = 0;%��ʼ������·������Ų���Ϊ0
        posi_opt = [];%��ʼ��posi_opt����
        relay = relay_main;       
        location_vehi_in_Rp = zeros(length(location_vehi(:,1)),4);%��ʼ��location_vehi_in_Rp���飬�������������������location_vehi����һ�£�location_vehi_in_Rp����Ϊlocation_vehi�����еĽڵ�λ��Rp���������еĲ���                
        [relay_road_index]=jyc_find_relayroad(locationmark,relay);
        if send_end(1,3) == 1
            broadcast_sign = 2;%���broadcast_signΪ2����ʾ�ڽ��������㲥�����broadcast_signΪ3���ǽ��е��Ƿ���㲥
        else
            broadcast_sign = 3;
        end
        mark_in_R = zeros(100,3);%��ʼ��mark_in_R���飨ͨ�ŷ�Χ�ڵ�·�α�ǵ���Ϣ���飩
        jump_mark_in_R = zeros(100,3);%��ʼ��jump_mark_in_R���飨ͨ�ŷ�Χ��·�α�ǵ��������飬��������·�α�ǵ��������Ϣ��
        mark_in_R_index = 1;%mark_in_R����ļ�������
        for i = 1:length(locationmark(:,1))
            if (sum((locationmark(i,1:2)-relay(1:2)).^2))^0.5/Linmap1m < R
               mark_in_R(mark_in_R_index,1) = i;
               mark_in_R(mark_in_R_index,2:3) = locationmark(i,1:2);
               mark_in_R_index = mark_in_R_index + 1;
            end
        end
        mark_in_R(mark_in_R(:,1)==0,:) = [];%ɾ��mark_in_R�����е�ȫ����
        
        for i = 1:length(mark_in_R(:,1)) %��ѭ������ɾ�����ϰ����谭��·�α�ǵ�
            sample_num = 100;%��ʼ����������Ϊ100
            det_X = (mark_in_R(i,2)-relay(1))/sample_num;
            det_Y = (mark_in_R(i,3)-relay(2))/sample_num;            
            for j = 1:sample_num
                temp_x = relay(1)+j*det_X;
                temp_y = relay(2)+j*det_Y;
                %   plot(temp_x,temp_y,'o');
                %   hold on
                x_data = ceil(temp_x);
                y_data = ceil(temp_y);
                if ima2(y_data,x_data) == 0    %��������Ӧͼ����������ֵΪ0��˵���˴�Ϊ�ϰ��˵���������ϰ����谭��
                    mark_in_R(i,:) = 0;    %����һ����������Ϣȫ����0
                    break;
                end
            end            
        end
        mark_in_R(mark_in_R(:,1)==0,:) = [];%ɾ��mark_in_R�����е�ȫ����
        jump_mark_in_R_index = 1; %·�α�ǵ����㴢�������������ʼ��Ϊ1
        for i = 1:length(mark_in_R(:,1))
            if i == length(mark_in_R(:,1)) && length(mark_in_R(:,1)) ~= 1
                break;
            elseif i == length(mark_in_R(:,1)) && length(mark_in_R(:,1)) == 1%���ͨ�ŷ�Χ��ֻ����һ��·�α�ǵ㣬��ô��һ��·�α�ǵ�������㿴
                jump_mark_in_R(jump_mark_in_R_index,:) = mark_in_R(i,:);%��������Ϣ����
                jump_mark_in_R_index = jump_mark_in_R_index + 1;
                break;
            end
            if abs(mark_in_R(i,1)-mark_in_R(i+1,1)) ~= 1 %�ҳ������д����·�α�ǵ�����
                jump_mark_in_R(jump_mark_in_R_index,:) = mark_in_R(i,:);%��������Ϣ����
                jump_mark_in_R(jump_mark_in_R_index+1,:) = mark_in_R(i+1,:);%��������Ϣ����
                jump_mark_in_R_index = jump_mark_in_R_index + 2;%������������������㣬�ʼ�2
            end
        end
        jump_mark_in_R(jump_mark_in_R_index,:)=mark_in_R(mark_in_R(:,1)==min(mark_in_R(:,1)),:);%ȡ��mark_in_R�д������С���·�α�ǵ����Ϣ��������������
        jump_mark_in_R_index = jump_mark_in_R_index+1;%��������������1
        jump_mark_in_R(jump_mark_in_R_index,:)=mark_in_R(mark_in_R(:,1)==max(mark_in_R(:,1)),:);%ȡ��mark_in_R�д���������·�α�ǵ����Ϣ��������������
        jump_mark_in_R(jump_mark_in_R(:,1)==0,:) = [];%ɾ���������飨jump_mark_in_R���е�ȫ����
        %��ʱ��������������1�ˣ���Ϊ�����Ѿ����������Ĵ��棬������Ҫ�Դ���������ظ��ĵ����ɾ��
        for i = 1:length(jump_mark_in_R(:,1))
            temp_jump_mark = jump_mark_in_R;
            temp_jump_mark(i,:) = [];%����һ��û��jump_mark_in_R(i,1)��Ϣ����ʱ���飬��ʣ�µ�������Ϣ���ҿ���û�к�jump_mark_in_R(i,1)��Ϣ�ظ�������
            if ~isempty(find(temp_jump_mark(:,1)==jump_mark_in_R(i,1), 1))%������������飨jump_mark_in_R�����ҵõ�jump_mark_in_R(i,1)���ظ�·�α�ǵ�
                jump_mark_in_R(i,:) = 0;%��jump_mark_in_R(i,1)����Ϣȫ����0
            end
        end
        jump_mark_in_R(jump_mark_in_R(:,1)==0,:) = [];%ɾ���������飨jump_mark_in_R���е�ȫ����
        %��������ʼ������������ѡ���м̽ڵ�����λ������·�ε�ĳһ��·�α�ǵ�
        for i = 1:length(jump_mark_in_R(:,1))
            if abs(jump_mark_in_R(i,1)-send_end(3)) == min(abs(jump_mark_in_R(:,1)-send_end(3)))
                posi_opt_mark = jump_mark_in_R(i,:);            
            end
        end
        %�����������м̽ڵ�����λ������·�ε�ĳһ��·�α�ǵ���10mΪ�ݽ��ҳ��м̽ڵ�����λ��posi_opt
        if posi_opt_mark(1) > send_end(3)%���·�α�ǵ���Ŵ�����Ϣ�����յ�����·��
            sample_num = ceil(((sum((locationmark(posi_opt_mark(1)-1,1:2)-locationmark(posi_opt_mark(1),1:2)).^2))^0.5/Linmap1m)/10);%�����10m�ݽ�������һ���ɲ����Ĵ���
            detX = (locationmark(posi_opt_mark(1)-1,1)-locationmark(posi_opt_mark(1),1))/sample_num;
            detY = (locationmark(posi_opt_mark(1)-1,2)-locationmark(posi_opt_mark(1),2))/sample_num;
            for i = 1:sample_num
                temppoint(1) = posi_opt_mark(2)+i*detX;
                temppoint(2) = posi_opt_mark(3)+i*detY;
                
                %��������һ�����ж�temppoint��relay�������Ƿ񴩹��ϰ��������cross_barΪ1����������cross_barΪ0
                sample_for_bar = 100;%��ʼ����������Ϊ100
                det_X = (temppoint(1)-relay(1))/sample_for_bar;
                det_Y = (temppoint(2)-relay(2))/sample_for_bar;
                for j = 1:sample_for_bar
                    cross_bar = 0;%�������ϰ���ı��λ��ʼ��Ϊ0
                    temp_x = relay(1)+j*det_X;
                    temp_y = relay(2)+j*det_Y;
                    %   plot(temp_x,temp_y,'o');
                    %   hold on
                    x_data = ceil(temp_x);
                    y_data = ceil(temp_y);
                    if ima2(y_data,x_data) == 0    %��������Ӧͼ����������ֵΪ0��˵��˳�����ߵݽ�ʱ�������ϰ��˵�����ߴ������ϰ���
                        cross_bar = 1;    %�������ϰ���ı��λ��1
                        break;
                    end
                end
                %�жϽ���                 
                if (sum((temppoint(1:2)-relay(1:2)).^2))^0.5/Linmap1m > R %�������ͨ�ŷ�Χ����˵����һλtemppoint��posi_opt��
                    if i ~= 1
                        posi_opt(1) = posi_opt_mark(2)+(i-1)*detX;   %�򵹻�ȥ�洢��һλ��������Ϣ
                        posi_opt(2) = posi_opt_mark(3)+(i-1)*detY;
                    else
                        posi_opt = posi_opt_mark(2:3);
                    end
                elseif cross_bar == 1%���temppoint��relay�������������ϰ����cross_barΪ1
                    if i ~= 1
                        posi_opt(1) = posi_opt_mark(2)+(i-1)*detX;   %�򵹻�ȥ�洢��һλ��������Ϣ
                        posi_opt(2) = posi_opt_mark(3)+(i-1)*detY;
                    else
                        posi_opt = posi_opt_mark(2:3);
                    end
                end
            end
        elseif posi_opt_mark(1) <= send_end(3)%���·�α�ǵ����С����Ϣ�����յ�����·��            
            sample_num = ceil(((sum((locationmark(posi_opt_mark(1)+1,1:2)-locationmark(posi_opt_mark(1),1:2)).^2))^0.5/Linmap1m)/10);%�����10m�ݽ�������һ���ɲ����Ĵ���
            detX = (locationmark(posi_opt_mark(1)+1,1)-locationmark(posi_opt_mark(1),1))/sample_num;
            detY = (locationmark(posi_opt_mark(1)+1,2)-locationmark(posi_opt_mark(1),2))/sample_num;
            for i = 1:sample_num
                temppoint(1) = posi_opt_mark(2)+i*detX;
                temppoint(2) = posi_opt_mark(3)+i*detY;
                
                %��������һ�����ж�temppoint��relay�������Ƿ񴩹��ϰ��������cross_barΪ1����������cross_barΪ0
                sample_for_bar = 100;%��ʼ����������Ϊ100
                det_X = (temppoint(1)-relay(1))/sample_for_bar;
                det_Y = (temppoint(2)-relay(2))/sample_for_bar;
                for j = 1:sample_for_bar
                    cross_bar = 0;%�������ϰ���ı��λ��ʼ��Ϊ0
                    temp_x = relay(1)+j*det_X;
                    temp_y = relay(2)+j*det_Y;
                    %   plot(temp_x,temp_y,'o');
                    %   hold on
                    x_data = ceil(temp_x);
                    y_data = ceil(temp_y);
                    if ima2(y_data,x_data) == 0    %��������Ӧͼ����������ֵΪ0��˵��˳�����ߵݽ�ʱ�������ϰ��˵�����ߴ������ϰ���
                        cross_bar = 1;    %�������ϰ���ı��λ��1
                        break;
                    end
                end
                %�жϽ���                 
                if (sum((temppoint(1:2)-relay(1:2)).^2))^0.5/Linmap1m > R %�������ͨ�ŷ�Χ����˵����һλtemppoint��posi_opt��
                    if i ~= 1
                        posi_opt(1) = posi_opt_mark(2)+(i-1)*detX;   %�򵹻�ȥ�洢��һλ��������Ϣ
                        posi_opt(2) = posi_opt_mark(3)+(i-1)*detY;
                    else
                        posi_opt = posi_opt_mark(2:3);
                    end
                elseif cross_bar == 1%���temppoint��relay�������������ϰ����cross_barΪ1
                    if i ~= 1
                        posi_opt(1) = posi_opt_mark(2)+(i-1)*detX;   %�򵹻�ȥ�洢��һλ��������Ϣ
                        posi_opt(2) = posi_opt_mark(3)+(i-1)*detY;
                    else
                        posi_opt = posi_opt_mark(2:3);
                    end
                end
            end                       
        end
             
    
        %����������������Rp��Χ�ڵĳ����ڵ�Ҳɸѡһ��              
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
            [vehi_road_index]=jyc_find_relayroad(locationmark,location_vehi_in_Rp(i,2:3));
            location_vehi_in_Rp(i,4) = vehi_road_index;
        end
        
        %���¶�Ŀ����Ϊ�˷�ֹ�м̽ڵ��ѡ����ֵ��˻���һ����Ȼ����ǰ����������һ��������ѭ��
        if relay_road_index>send_end(3)
            for i = 1:length(location_vehi_in_Rp(:,1))
                if location_vehi_in_Rp(i,4)>=relay_road_index  %�ô��ڵ�����Ϊ�˽����ܳ��ֵ���ͬһ·���Ϸ�������ѡ���������
                   location_vehi_in_Rp(i,:) = 0;
                end
            end
        elseif relay_road_index<send_end(3)
            for i = 1:length(location_vehi_in_Rp(:,1))
                if location_vehi_in_Rp(i,4)<=relay_road_index  %��С�ڵ�����Ϊ�˽����ܳ��ֵ���ͬһ·���Ϸ�������ѡ���������
                   location_vehi_in_Rp(i,:) = 0;                    
                end
            end            
        end
        %�����˴�ʩ���˽���
        location_vehi_in_Rp(location_vehi_in_Rp(:,1)==0,:) = [];%��location_vehi_in_Rp������ȫ����ȥ�� 
        if isempty(location_vehi_in_Rp)==0%���location_vehi_in_Rp���鲻Ϊ��
            %�ҳ�location_vehi_in_Rp�������ظ���Ԫ�ؽ���ɾ��
            for i = 1:length(location_vehi_in_Rp(:,1))
                for j = 1:length(location_vehi_in_Rp(:,1))
                    if i~=j && location_vehi_in_Rp(i,2) == location_vehi_in_Rp(j,2) && location_vehi_in_Rp(i,3) == location_vehi_in_Rp(j,3)
                        location_vehi_in_Rp(j,:) = 0;
                    end
                end
            end
            location_vehi_in_Rp(location_vehi_in_Rp(:,1)==0,:) = [];%��location_vehi_in_Rp������ȫ����ȥ�� 
        end
        
        if broadcast_sign == 2
            if isempty(posi_opt) && posi_opt_mark(1,2) == send_end(1,1) && posi_opt_mark(1,3) == send_end(1,2)%���posi_optΪ�գ�������ѡ��������λ������·�α�ǵ㣨posi_opt_mark������send_end�㣬��ô��send_end�㶨Ϊ����λ�ã�posi_opt��
                posi_opt = send_end(1,1:2);
            end
%             plot(posi_opt(1),posi_opt(2),'*','LineWidth',1,'MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',8);%�����㲥��Popt����
%             hold on
        elseif broadcast_sign == 3
            posi_opt = [0 0];
        end
    end
