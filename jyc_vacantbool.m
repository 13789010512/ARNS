function [vacant0,vacant,re_symbol]=jyc_vacantbool(n_relay,locationmark_nocover,location_relay,send_start,locationmark)

 %�����ж�X������send��relay�м��·���Ƿ񱻹㲥�������ж��Ƿ���ֿհ�����  
         vacant = zeros(length(locationmark_nocover(:,1)),4);%��ʼ��4�е�vacant����һ���������洢x,y���꣬�����д�0������d_vacant��û�취d_vacant����ȡ����Ҫ����ά�ĽǶ���ã�vacant��z��Ĭ��Ϊ0,6.26�������������ʱû�ã��������д洢vacant��·�����
         re_symbol = 0;   %��ʼ������㲥��־λΪ0
         i_count = 1;     %vacant����洢����
         if n_relay>1
             [relay_road_index]=jyc_find_relayroad(locationmark,location_relay(n_relay-1,2:3));
         end
         for i = 1:length(locationmark_nocover(:,1))%�����￪ʼ�������vacant0
             if n_relay == 2 && relay_road_index<locationmark_nocover(i,1) && locationmark_nocover(i,1)<send_start(3)
                 vacant(i_count,1:2) = locationmark_nocover(i,2:3);
                 vacant(i_count,4) = locationmark_nocover(i,1);
                 i_count = i_count+1;
             elseif n_relay>2
                 before_relay_road_index = jyc_find_relayroad(locationmark,location_relay(n_relay-2,2:3));
                 if relay_road_index<locationmark_nocover(i,1) && locationmark_nocover(i,1)<before_relay_road_index
                     vacant(i_count,1:2) = locationmark_nocover(i,2:3);
                     vacant(i_count,4) = locationmark_nocover(i,1);
                     i_count = i_count+1;
                 end
             end
         end          
         vacant0 = vacant;
         vacant0(:,3)=[];
         vacant0(vacant(:,1)==0,:) = [];%�������������forѭ��������ѡ����հ������е�·�α�ǵ�      
         if isempty(vacant0)==0
            re_symbol = 1;
         end
end