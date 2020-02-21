function [locationmark] = findroad (junction,end_relay_junction,branchset,last_relay,locationjunction)
for i = 1:26
    name1 = str2num(branchset{1,i}(1:2));
    name2 = str2num(branchset{1,i}(4:5));
    if name1 == junction && name2 == end_relay_junction
        locationmark_temp = branchset{2,i};
        break;
    elseif name2 == junction && name1 == end_relay_junction
        locationmark_temp = branchset{2,i};
        break;
    end
end
%���������м̽ڵ�ѡ������ж��Ǵ�·�α�Ŵ�ĵط���·�α��С�ĵط�������Ϣ�㲥����������Ҫ��һ��locationmark_temp�и�·�ε�����
dis1 = (sum((locationmark_temp(1,2:3)-last_relay).^2))^0.5;
dis2 = (sum((locationmark_temp(length(locationmark_temp(:,1)),2:3)-last_relay).^2))^0.5;
if dis1>dis2%���locationmark_temp��·�α�Ŵ�ĵ����last_relay������ô���õ����
    locationmark = locationmark_temp;    
else%���locationmark_temp��·�α��С�ĵ����last_relay������ô������Ҫ�ߵ����
    locationmark = flipud(locationmark_temp);
    locationmark(:,1) = 1:length(locationmark(:,1));
end
%�����������ٶ�locationmark��ӹ�һ�£�ʹ����������ǵ��м̽ڵ�ѡ�����
%���ȣ����ǽ�last_relay���뵽locationmark�У���Ϊ·�α�������Ǹ���
if locationmark(length(locationmark(:,1)),2)~=last_relay(1,1)||locationmark(length(locationmark(:,1)),3)~=last_relay(1,2)%���locationmark·�α�������Ǹ��㲻��last_relay����ô���ǰ�last_relay���뵽locationmark���һ�У���Ϊ�������·�α�ǵ�
    index = length(locationmark(:,1))+1;
    locationmark(index,1) = index;
    locationmark(index,2:3) = last_relay;
end
%Ȼ�����ǽ�Ŀ��junction�����locationmark����ĵ�һ���У���Ϊ·�α����С���Ǹ��㣬ԭlocationmark����Ԫ��ͳͳ������һλ
if locationmark(1,2:3)~=locationjunction(junction,2:3)
    locationmark = [1 locationjunction(junction,2:3); locationmark(1:end,:)];
    locationmark(2:end,1) = 2:length(locationmark(:,1));
end
locationmark(:,1)=[];
