function [t_partition,bound_left,bound_right,inform_vehi]=uniform_partition(relay_y,range,inform_vehi,n_uniform_parti,n_uniform_itera,relay)

%% partition 
%% t_partition:partition��ʱ��bound_left,bound_right�ֱ�Ϊһ��one hop��partition�����ұ߽�;
global t_slot;

  t_partition = 0;
  %vehi_one_hop_index = find(relay_y<=inform_vehi(:,4)<relay_y+R);
  n_seg_min = n_uniform_parti^n_uniform_itera;% �ڴ��䷶Χ����Сsegmennt�ĸ���
  w_seg_min = range/n_seg_min; % ��Сsegmennt�Ŀ��
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
      if inform_vehi(i,1)==0 && inform_vehi(i,4)>range%�����������Ϊ����R��Χ������û�л��������ĳ����ڵ㣬ͳһ���ڳ�ʼ�ұ߽�֮�⣬��ͨ��С������
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

            