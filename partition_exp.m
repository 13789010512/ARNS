function relay = partition_exp(point_opti,Rp,inform_vehi_part,n_itera,n_parti,A,Linmap1m)

%% partition :point_opti:期望找到node最优的位置；Rp：分割范围
%% relay;中继节点信息

  n_slot_partition=0;
   t_partition_mini=0;
  
  %vehi_one_hop_index = find(relay_y<=inform_vehi(:,4)<relay_y+R);

  n_seg_min = (3^(n_parti-1))^n_itera;% 在传输范围内最小segmennt的个数
  [width_segm,bound_segm]=width_segment(n_parti,n_itera,A);
%   width_segm=width_segm(length(width_segm):-1:1);
%   bound_segm=1-bound_segm;
%   w_seg_min = range/n_seg_min; % 最小segmennt的宽度

  bound_vector = bound_segm*Rp;
  bound_segment_in_one_itera = zeros(n_parti+1,1);
  bound_left = 0;%
  bound_right = n_parti^n_itera;%
  
  bound_vector=bound_vector(length(bound_vector):-1:1);
      EuDistance=((inform_vehi_part(:,2)-point_opti(1)).^2+(inform_vehi_part(:,3)-point_opti(2)).^2).^0.5/Linmap1m;
  for i=1:length(bound_vector)-1
  
      vehi_one_hop_index = find((bound_vector(i)<=EuDistance)&(EuDistance<bound_vector(i+1)));%此处返回的是因为调用，新的infor_intersection下标，还是以前对应的
      inform_vehi_part(vehi_one_hop_index,4) = i;%标记vehicle在哪个segment，最小的segement的标号最小
      clear vehi_one_hop_index;
  end
   inform_vehi_part(find(inform_vehi_part(:,4)==0),4)=n_parti^n_itera;
   candi_vehi_index = find(inform_vehi_part(:,4)==min(inform_vehi_part(:,4)));
   candi_vehi = zeros(length(candi_vehi_index),4) ;
   relay = zeros(1,2) ;
   candi_vehi= inform_vehi_part(candi_vehi_index,:);
   if isempty(candi_vehi_index)
       a=1;
   end
   relay = candi_vehi(randi(length(candi_vehi_index(:,1)),1,1),2:3);
  