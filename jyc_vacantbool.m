function [vacant0,vacant,re_symbol]=jyc_vacantbool(n_relay,locationmark_nocover,location_relay,send_start,locationmark)

 %根据判断X坐标在send和relay中间的路段是否被广播覆盖来判断是否出现空白区域  
         vacant = zeros(length(locationmark_nocover(:,1)),4);%初始化4列的vacant，第一二列用来存储x,y坐标，第三列存0用来求d_vacant（没办法d_vacant的求取必须要从三维的角度求得，vacant的z轴默认为0,6.26修正后第三列暂时没用），第四列存储vacant的路段序号
         re_symbol = 0;   %初始化反向广播标志位为0
         i_count = 1;     %vacant数组存储计数
         if n_relay>1
             [relay_road_index]=jyc_find_relayroad(locationmark,location_relay(n_relay-1,2:3));
         end
         for i = 1:length(locationmark_nocover(:,1))%从这里开始至下面的vacant0
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
         vacant0(vacant(:,1)==0,:) = [];%从这里至上面的for循环，都是选择出空白区域中的路段标记点      
         if isempty(vacant0)==0
            re_symbol = 1;
         end
end