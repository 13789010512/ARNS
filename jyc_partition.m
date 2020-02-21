function [ t_partition,t_partition_mini,bound_left,bound_right,inform_vehi]=jyc_partition(relay_y,relay,R,inform_vehi,n_itera,n_parti)

%% partition 
%% t_partition:partition��ʱ��bound_left,bound_right�ֱ�Ϊһ��one hop��partition�����ұ߽�;
global  t_slot ;

  t_partition = 0;
  %vehi_one_hop_index = find(relay_y<=inform_vehi(:,4)<relay_y+R);
  n_seg_min = (2^(n_parti-1))^n_itera;% �ڴ��䷶Χ����Сsegmennt�ĸ���
  w_seg_min = R/n_seg_min; % ��Сsegmennt�Ŀ��
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
      if inform_vehi(i,1)==0 && inform_vehi(i,4)>R%�����������Ϊ����R��Χ������û�л��������ĳ����ڵ㣬ͳһ���ڳ�ʼ�ұ߽�֮�⣬��ͨ��С������
          inform_vehi(i,1) = bound_right+1;
      end
  end

  for i = 1:length(inform_vehi(:,1))%��inform_vehi���ҵ�relay���ڵ�λ��
      if inform_vehi(i,4)==relay(1,2)
          relay(1,1) = inform_vehi(i,1);%��relayӦ�е�С���α�Ÿ�ֵ��relay�ĵ�һ���У�ԭ�����ǵ�һ���Ǵ�relay��X����ģ�������������X�������û�ã��������Ǿ���ʱ��relay�ĵ�һ�д�һ��С���α��
          break;
      end
  end
  
  for j=1:n_itera  %ʵ�ֵ�ֻ��3-nary partition
      bound_segment_in_one_itera(1) = bound_left;
      bound_segment_in_one_itera(n_parti+1) = bound_right;
      for i=n_parti:-1:2
          if i == n_parti
              bound_segment_in_one_itera(i) = bound_segment_in_one_itera(i+1)-fix((bound_segment_in_one_itera(i+1)-bound_segment_in_one_itera(1))/2);%inner segment�Ľӽ��ұ߽��min-segment��Ӧ�ı��
          else
              bound_segment_in_one_itera(i) = bound_segment_in_one_itera(i+1)-fix((bound_segment_in_one_itera(i+2)-bound_segment_in_one_itera(i+1))/2);%inner segment�Ľӽ��ұ߽��min-segment��Ӧ�ı��
          end
      end
       t=1;
       for i=1:n_parti
           if sum((bound_segment_in_one_itera(i)<relay(1,1))&(relay(1,1)<=bound_segment_in_one_itera(i+1)))%����ҵ�relay����С��������Ҫ�ķ�������
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
  t_partition=(t_partition+1)*t_slot;%��󽫷�����������ÿ�η�������ʱ�ó����յķ�����ʱ
  t_partition_mini=(t_partition+1)*t_slot;
            