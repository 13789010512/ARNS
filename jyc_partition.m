function [ t_partition,t_partition_mini,bound_left,bound_right,inform_vehi]=jyc_partition(relay_y,relay,R,inform_vehi,n_itera,n_parti)

%% partition 
%% t_partition:partition延时；bound_left,bound_right分别为一次one hop中partition后左右边界;
global  t_slot ;

  t_partition = 0;
  %vehi_one_hop_index = find(relay_y<=inform_vehi(:,4)<relay_y+R);
  n_seg_min = (2^(n_parti-1))^n_itera;% 在传输范围内最小segmennt的个数
  w_seg_min = R/n_seg_min; % 最小segmennt的宽度
  bound_vector = relay_y:w_seg_min:relay_y+R;
  bound_segment_in_one_itera = zeros(n_parti+1,1);
  bound_left = 0;
  bound_right = (2^(n_parti-1))^n_itera;
  
  for i=1:length(bound_vector)-1
      vehi_one_hop_index = find((bound_vector(i)<=inform_vehi(:,4))&(inform_vehi(:,4)<=bound_vector(i+1)));
      inform_vehi(vehi_one_hop_index,1) = i;
      clear vehi_one_hop_index;
  end
  
  for i = 1:length(inform_vehi(:,1))
      if inform_vehi(i,1)==0 && inform_vehi(i,4)>R%如果出现了因为超出R范围而导致没有划定分区的车辆节点，统一归于初始右边界之外，即通信小区间外
          inform_vehi(i,1) = bound_right+1;
      end
  end

  for i = 1:length(inform_vehi(:,1))%从inform_vehi中找到relay所在的位置
      if inform_vehi(i,4)==relay(1,2)
          relay(1,1) = inform_vehi(i,1);%将relay应有的小区段编号赋值至relay的第一列中，原本我们第一列是存relay的X坐标的，但是我们现在X坐标基本没用，所以我们就暂时用relay的第一列存一下小区段编号
          break;
      end
  end
  
  for j=1:n_itera  %实现的只是3-nary partition
      bound_segment_in_one_itera(1) = bound_left;
      bound_segment_in_one_itera(n_parti+1) = bound_right;
      for i=n_parti:-1:2
          if i == n_parti
              bound_segment_in_one_itera(i) = bound_segment_in_one_itera(i+1)-fix((bound_segment_in_one_itera(i+1)-bound_segment_in_one_itera(1))/2);%inner segment的接近右边界的min-segment对应的标号
          else
              bound_segment_in_one_itera(i) = bound_segment_in_one_itera(i+1)-fix((bound_segment_in_one_itera(i+2)-bound_segment_in_one_itera(i+1))/2);%inner segment的接近右边界的min-segment对应的标号
          end
      end
       t=1;
       for i=1:n_parti
           if sum((bound_segment_in_one_itera(i)<relay(1,1))&(relay(1,1)<=bound_segment_in_one_itera(i+1)))%算出找到relay所在小区段所需要的分区次数
               bound_left = bound_segment_in_one_itera(i);
               bound_right = bound_segment_in_one_itera(i+1);
               t_partition = t_partition+t;
               break;
           end
           t=t+1;
       end
       if t==n_parti
           t_partition=t_partition-1;
       end
  end
  t_partition=(t_partition+1)*t_slot;%最后将分区次数乘以每次分区的延时得出最终的分区延时
  t_partition_mini=(t_partition+1)*t_slot;
            