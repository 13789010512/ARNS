function [location_relay,re_t,num_vehi,location_vehi,location_re_relay,n_re_relay,has_cov_symbol,PDR,last_t]=jyc_bar_3P3B_reverse_relay_selection(location_vehi,n_relay,locationmark,R,Linmap1m,num_vehi,location_relay,N_iter,N_part,A,send_start,t,re_t,location_re_relay,n_re_relay,vacant0,vacant,re_symbol,relay_main,has_cov_symbol,T,ima2)
    %����㲥���
        global density_EM CW_CTB;   
    %������Ҫ��鷴��㲥�հ������е�·�α�ǵ��Ƿ��������ǹ�
        recover_index_set = zeros(length(vacant0(:,1)),1);%��ʼ���ظ����Ǳ������Ϊlength(vacant0(:,1))�У�1�е�����
        %���������vacant0�������˵��vacant���е�·�α�ǵ��Ƿ�����location_relay�м̽ڵ��location_re_relay����㲥�м̽ڵ㸲�ǹ���������ǹ�һ�Σ���recover������1���ظ�����
        for i = 1:length(vacant0(:,1))
            recover = 0;%��ʼ���ظ����ǲ���Ϊ0           
            for j = 1:length(location_relay(:,1))
                if location_relay(j,1)==0
                    break;
                elseif (sum((location_relay(j,2:3)-vacant0(i,1:2)).^2))^0.5/Linmap1m<R%����հ������еĵ���location_relay�����е������м̽ڵ��е�ͨ�ŷ�Χ�ڣ������Ƿ��ϰ����谭��
                    recover = 1;%��ʼ���ظ����Ǳ��λΪ1
                    %����Ϊ����Ƿ��ϰ����谭����
                    sample_num = 100;%��ʼ����������Ϊ100
                    det_X = (vacant0(i,1)-location_relay(j,2))/sample_num;
                    det_Y = (vacant0(i,2)-location_relay(j,3))/sample_num;
                    for n = 1:sample_num
                        temp_x = location_relay(j,2)+n*det_X;
                        temp_y = location_relay(j,3)+n*det_Y;
                        x_data = ceil(temp_x);
                        y_data = ceil(temp_y);
                        if ima2(y_data,x_data) == 0    %��������Ӧͼ����������ֵΪ0��˵���˴�Ϊ�ϰ��˵���������ϰ����谭��                            
                            recover = 0;%���ظ����Ǳ��λ��0����ʾ��ʵû���ظ�����
                            break;
                        end
                    end    
                    %�����ϣ�������谭�����ظ����Ǳ�־λ��0
                end
            end
            
            for j = 1:length(location_re_relay(:,1))             
                if location_re_relay(j,1)==0
                    break;
                elseif (sum((location_re_relay(j,2:3)-vacant0(i,1:2)).^2))^0.5/Linmap1m<R%����հ������еĵ���location_re_relay�����е������м̽ڵ��е�ͨ�ŷ�Χ�ڣ������Ƿ��ϰ����谭��                  
                    recover = 1;%��ʼ���ظ����Ǳ��λΪ1
                    %����Ϊ����Ƿ��ϰ����谭����
                    sample_num = 100;%��ʼ����������Ϊ100
                    det_X = (vacant0(i,1)-location_re_relay(j,2))/sample_num;
                    det_Y = (vacant0(i,2)-location_re_relay(j,3))/sample_num;
                    for n = 1:sample_num
                        temp_x = location_re_relay(j,2)+n*det_X;
                        temp_y = location_re_relay(j,3)+n*det_Y;
                        x_data = ceil(temp_x);
                        y_data = ceil(temp_y);
                        if ima2(y_data,x_data) == 0    %��������Ӧͼ����������ֵΪ0��˵���˴�Ϊ�ϰ��˵���������ϰ����谭��                            
                            recover = 0;%���ظ����Ǳ��λ��0����ʾ��ʵû���ظ�����
                            break;
                        end
                    end 
                    %�����ϣ�������谭�����ظ����Ǳ�־λ��0
                end
            end           
            recover_index_set(i,1) = recover;%��recover��ֵ����������
        end
        
        if isempty(find(recover_index_set==0, 1))%���Ѱ��recover_index_set���ظ����Ǳ�����飩��δ�����ǹ���·�α�ǵ㷵�صĽ���ǿգ���recover=0��һ����û�У�ȫ��recover=1�ظ����ǣ�������ʾvacant0����vacant���е�����·�ε㶼�������ǹ�һ����
            re_symbol=0;%�򽫷���㲥�����0�������뷴��㲥
            has_cov_symbol = 1;%1��ʾ�հ������Ѿ������ǹ���
        end
    %�����հ������е�·�ε��Ƿ��������ǵļ��
    
    if re_symbol == 1 %�������㲥��־λΪ1�����뷴��㲥
        r = length(vacant0(:,1));%vacant����vacant0���������Ч����������������
        r_end = length(vacant(:,1));
        for i = r+1:r_end
            vacant(r+1,:)=[];%�����vacant�����е�r��֮�������
        end
        
        if re_t == 0
            re_t = t;  %�������㲥�ļ�ʱre_tΪ0����ʾ֮ǰû�н��������㲥�����ʼ������㲥�ļ�ʱ��t��ʼ
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
%         plot(re_send_end(1),re_send_end(2),'o','LineWidth',1,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',6);%�������㲥���յ�λ��
%         hold on
        re_send_start = relay_main;        
        re_relay = re_send_start;%re_relay��ʼֵΪre_send_start
        re_relayend = re_send_end;%ȷ������㲥�յ�λ��        
        n_re_relay = 1;
        PDR = [];%��ʼ��PDRΪ��
        %����㲥������������㲥����һ��,���¶��Ƿ���㲥�龰
        while isempty(vacant0)==0 &&  re_t <= T
            re_relay_candi = zeros(length(location_vehi(:,1)),3);%��ʼ��re_relay_candi������㲥�м̽ڵ㱸�ýڵ����Ϊlength(location_vehi(:,1))�У�3�е�ȫ�����
            sender = re_relay;
            re_relay_location(n_re_relay,1) = n_re_relay;
            re_relay_location(n_re_relay,2:3) = re_relay;
            re_relay_location(n_re_relay,4) = 2;%re_relay_location�������з���㲥�м̽ڵ㣬1����ţ� 2��3��(x,y)�� 4������(0���ֿհ�����1ֱ����2�����3ʮ��·��)
            continu = 1;%��ʼ���Ƿ�������־λ��1��1��������0������
            [~,location_vehi_in_Rp]=jyc_bar_posi_opt(location_vehi,locationmark,R,Linmap1m,re_relay,re_send_end,ima2);     %ֻ����Ѱ������λ�ú���ɸѡͨ�ŷ�Χ�ڳ����ڵ�Ĺ���
            [re_relay_road_index]=jyc_find_relayroad(locationmark,re_relay);                      
            for i = 1:length(vacant0(:,1))-1
                if vacant0(i,3)+1 ~= vacant0(i+1,3)%���������
                    continu = 0;%��������־λ��0
                    break;
                else%�������
                    continu = 1;%��������־λ��1
                end
            end
            
            for j = 1:length(locationmark(:,1))
                if (sum((re_relay-locationmark(j,1:2)).^2))^0.5/Linmap1m>R && j>re_relay_road_index%�ҳ�·�α���ϳ���re_relay�ĵ�һ����re_relayͨ�ŷ�Χ���·�α�ǵ�
                    if j<=re_send_end(1,3)%�����·�α�ǵ���С�ڵ���re_send_end����·�α�ţ����ֱ��ѡ����Ϊ����λ������·��
                        for n = re_relay_road_index+1:j-1
                            sample_num = 100;%��������ȡ100���ж��Ƿ��ϰ����谭
                            for m = 1:sample_num
                                detx = (locationmark(n,1)-re_relay(1,1))/sample_num;%���detx
                                dety = (locationmark(n,2)-re_relay(1,2))/sample_num;%���dety
                                x_data = ceil(re_relay(1,1)+m*detx);
                                y_data = ceil(re_relay(1,2)+m*dety);%���ȡ��x_data,y_data��ֵ��׼������ima2�в鿴�Ƿ��ϰ����赲
                                if ima2(y_data,x_data) == 0%��������Ӧͼ����������ֵΪ0��˵���˴�Ϊ�ϰ��˵��vacant0�е�·�α�ǵ㱻�ϰ����谭��
                                    break;
                                elseif m == sample_num && ima2(y_data,x_data) ~= 0
                                    posi_opt_mark = n;                                   
                                end
                            end
                        end
                        break;
                    else
                        for n = re_relay_road_index+1:re_send_end(1,3)                           
                            sample_num = 100;%��������ȡ100���ж��Ƿ��ϰ����谭
                            for m = 1:sample_num
                                detx = (locationmark(n,1)-re_relay(1,1))/sample_num;%���detx
                                dety = (locationmark(n,2)-re_relay(1,2))/sample_num;%���dety
                                x_data = ceil(re_relay(1,1)+m*detx);
                                y_data = ceil(re_relay(1,2)+m*dety);%���ȡ��x_data,y_data��ֵ��׼������ima2�в鿴�Ƿ��ϰ����赲
                                if ima2(y_data,x_data) == 0%��������Ӧͼ����������ֵΪ0��˵���˴�Ϊ�ϰ��˵��vacant0�е�·�α�ǵ㱻�ϰ����谭��
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
                                    
            sample_num = ceil(((sum((locationmark(posi_opt_mark,1:2)-locationmark(posi_opt_mark+1,1:2)).^2))^0.5/Linmap1m)/10);%ÿ10�׳������ɳ����Ĵ���(���ȡ��)
            detx = (locationmark(posi_opt_mark+1,1)-locationmark(posi_opt_mark,1))/sample_num;%x������
            dety = (locationmark(posi_opt_mark+1,2)-locationmark(posi_opt_mark,2))/sample_num;%y������
            for n = 1:sample_num
                posi_opt(1,1) = locationmark(posi_opt_mark,1)+n*detx;
                posi_opt(1,2) = locationmark(posi_opt_mark,2)+n*dety;
                
                cross_bar = [0 0];%��ʼ���ϰ����谭���Ϊ[0 0],��һ������ʾ��һ�����Ƿ��ϰ����谭�ı�־���ڶ�������ʾ��ǰ���Ƿ��ϰ����谭
                sample_num = 100;%��������ȡ100���ж��Ƿ��ϰ����谭
                for m = 1:sample_num
                    detx = (posi_opt(1,1)-re_relay(1,1))/sample_num;%���detx
                    dety = (posi_opt(1,2)-re_relay(1,2))/sample_num;%���dety
                    x_data = ceil(re_relay(1,1)+m*detx);
                    y_data = ceil(re_relay(1,2)+m*dety);%���ȡ��x_data,y_data��ֵ��׼������ima2�в鿴�Ƿ��ϰ����赲
                    if ima2(y_data,x_data) == 0%��������Ӧͼ����������ֵΪ0��˵���˴�Ϊ�ϰ��˵��vacant0�е�·�α�ǵ㱻�ϰ����谭��
                        cross_bar(1,1) = cross_bar(1,2);
                        cross_bar(1,2) = 1;
                        break;                   
                    end
                end
                if (sum((re_relay-posi_opt).^2))^0.5/Linmap1m>R%���posi_opt����ͨ�ű߽磬��ôѡȡ��һ��posi_opt��Ϊ������posi_opt
                    posi_opt(1,1) = locationmark(posi_opt_mark,1)+(n-1)*detx;
                    posi_opt(1,2) = locationmark(posi_opt_mark,2)+(n-1)*dety;
                    break;
                else%���posi_optû�г���ͨ�ű߽�
                    if cross_bar(1,2)==1 && cross_bar(1,1)==0%�����һ��posi_optδ���ϰ����谭������ǰ��posi_opt���ϰ����谭��������ѡ��һ��posi_opt��Ϊ������posi_opt
                        posi_opt(1,1) = locationmark(posi_opt_mark,1)+(n-1)*detx;
                        posi_opt(1,2) = locationmark(posi_opt_mark,2)+(n-1)*dety;
                        break;
                    end
                end
            end
                    
           % if continu == 1%�������
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
               re_relay_candi(re_relay_candi(:,3)==0,:)=[];%��re_relay_candiȫ����ɾ��
               if isempty(re_relay_candi)
                   re_relay = posi_opt;
               else
                   re_relay = re_relay_candi(re_relay_candi(:,3)==max(re_relay_candi(:,3)),1:2);
               end
           end
           if length(re_relay(:,1))>1
               dis_relay_popt = zeros(length(re_relay(:,1)),1);%��ʼ��dis_relay_popt����������ŷ���relay��Popt��ľ���
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
            
            %����vacant0������
            for j = 1:length(vacant0(:,1))
                crossbar = 0;%�����ϰ����־λ��ʼ����0��0��û�д�����1��ʾ�����������ϰ�����˼��·�α�ǵ���re_relay�����ߴ����ϰ����ʾ·�α�ǵ㱻�ϰ����谭��
                if (sum((re_relay-vacant0(j,1:2)).^2))^0.5/Linmap1m<=R
                    sample_num = ceil(((sum((re_relay-vacant0(j,1:2)).^2))^0.5/Linmap1m)/10);%���10m�ݽ��ĳ�������,���ȡ��
                    for n = 1:sample_num
                        detx = (vacant0(j,1)-re_relay(1,1))/sample_num;%���detx
                        dety = (vacant0(j,2)-re_relay(1,2))/sample_num;%���dety
                        x_data = ceil(re_relay(1,1)+n*detx);
                        y_data = ceil(re_relay(1,2)+n*dety);%���ȡ��x_data,y_data��ֵ��׼������ima2�в鿴�Ƿ��ϰ����赲
                        if ima2(y_data,x_data) == 0%��������Ӧͼ����������ֵΪ0��˵���˴�Ϊ�ϰ��˵��vacant0�е�·�α�ǵ㱻�ϰ����谭��
                            crossbar = 1;%�����ϰ����־λ��1
                            break;
                        end
                    end
                    if crossbar == 0%��������ϰ����־λ��0��˵��·�α�ǵ�û�б��ϰ����谭
                        vacant0(j,:) = 0;
                    end
                end                                 
            end
            vacant0(vacant0(:,3)==0,:) = [];%�����vacant0�е�ȫ���У���ʾ���vacant0�ĸ���    
            [delay_one_hop_average,PDR_part,re_relay] = jyc_3P3B_delay_calcu (location_vehi_in_Rp,density_EM,N_iter,N_part,CW_CTB,sender,posi_opt,re_relay);
            re_t = re_t+delay_one_hop_average;
            PDR = [PDR;PDR_part];%whileѭ�������޼�PDR_part��PDR������
            
            %����vacant0���������       
            location_re_relay(n_re_relay,1) = n_re_relay;
            location_re_relay(n_re_relay,2:3) = re_relay;
            location_re_relay(n_re_relay,4) = 2;%location_re_relay(n_re_relay,4) = 2 ��ʾ��������µ��м̽ڵ�ѡ��
            n_re_relay = n_re_relay+1;

        end
        if (sum((re_relay(1:2)-re_relayend(1:2)).^2))^0.5/Linmap1m <= R
            re_relay = re_relayend;
%             plot(re_relay(1),re_relay(2),'v','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize',6);
%             hold on
            location_re_relay(n_re_relay,1) = n_re_relay;
            location_re_relay(n_re_relay,2:3) = re_relay(1:2);
            location_re_relay(n_re_relay,4) = 2;%location_re_relay(n_re_relay,4) = 2 ��ʾ��������µ��м̽ڵ�ѡ��
            n_re_relay = n_re_relay+1;
            has_cov_symbol = 1;%���ڷ���㲥��while�󣬱�ʾ�Ѿ����հ����򸲸�
           % re_relay_main = re_relay;
        end 
    end
    last_t = delay_one_hop_average;
end