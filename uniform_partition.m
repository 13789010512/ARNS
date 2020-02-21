function [t_partition,bound_left,bound_right,inform_vehi]=uniform_partition(relay_y,range,inform_vehi,n_uniform_parti,n_uniform_itera,relay)

%% partition 
%% t_partition:partition延时；bound_left,bound_right分别为一次one hop中partition后左右边界;
global t_slot;

  t_partition = 0;
  %vehi_one_hop_index = find(relay_y<=inform_vehi(:,4)<relay_y+R);
  n_seg_min = n_uniform_parti^n_uniform_itera;% 在传输范围内最小segmennt的个数
  w_seg_min = range/n_seg_min; % 最小segmennt的宽度
  bound_vector = relay_y:w_seg_min:relay_y+range;
  bound_segment_in_one_itera = zeros(n_uniform_parti+1,1);
  bound_left = 0;
  bound_right =  length( bound_vector)-1;
  
  
  for i=1:length(bound_vector)-1
      vehi_one_hop_index = (bound_vector(i)<=inform_vehi(:,4))&(inform_vehi(:,4)<=bound_vector(i+1));
      inform_vehi(vehi_one_hop_index,1) = i;
      if bound_vector(i)<=relay(1,2) && relay(1,2)<=bound_vector(i+1)
          relay(1,1) = i;
      end
      clear vehi_one_hop_index;
  end
  for i = 1:length(inform_vehi(:,1))
      if inform_vehi(i,1)==0 && inform_vehi(i,4)>range%如果出现了因为超出R范围而导致没有划定分区的车辆节点，统一归于初始右边界之外，即通信小区间外
          inform_vehi(i,1) = bound_right+1;
      end
  end
   
  for j=1:n_uniform_itera  

      bound_segment_in_one_itera = bound_left:((bound_right-bound_left)/n_uniform_parti):bound_right;
       
       t=1;
       for i=1:n_uniform_parti   
           if sum((bound_segment_in_one_itera(i)<relay(1,1))&(relay(1,1)<=bound_segment_in_one_itera(i+1)))
               bound_left = bound_segment_in_one_itera(i);
               bound_right = bound_segment_in_one_itera(i+1);
               t_partition = t_partition+t;
               break;
           end
           t=t+1;
       end
          
  end
  t_partition=(t_partition+1)*t_slot;

            