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
%由于我们中继节点选择代码中都是从路段编号大的地方往路段编号小的地方进行消息广播，故我们需要调一下locationmark_temp中各路段点的序号
dis1 = (sum((locationmark_temp(1,2:3)-last_relay).^2))^0.5;
dis2 = (sum((locationmark_temp(length(locationmark_temp(:,1)),2:3)-last_relay).^2))^0.5;
if dis1>dis2%如果locationmark_temp中路段标号大的点距离last_relay近，那么不用调序号
    locationmark = locationmark_temp;    
else%如果locationmark_temp中路段标号小的点距离last_relay近，那么我们需要颠倒序号
    locationmark = flipud(locationmark_temp);
    locationmark(:,1) = 1:length(locationmark(:,1));
end
%接下来我们再对locationmark多加工一下，使其更符合我们的中继节点选择代码
%首先，我们将last_relay加入到locationmark中，作为路段标号最大的那个点
if locationmark(length(locationmark(:,1)),2)~=last_relay(1,1)||locationmark(length(locationmark(:,1)),3)~=last_relay(1,2)%如果locationmark路段标号最大的那个点不是last_relay，那么我们把last_relay加入到locationmark最后一行，作为编号最大的路段标记点
    index = length(locationmark(:,1))+1;
    locationmark(index,1) = index;
    locationmark(index,2:3) = last_relay;
end
%然后，我们将目标junction点加在locationmark数组的第一行中，作为路段标号最小的那个点，原locationmark数组元素统统往后移一位
if locationmark(1,2:3)~=locationjunction(junction,2:3)
    locationmark = [1 locationjunction(junction,2:3); locationmark(1:end,:)];
    locationmark(2:end,1) = 2:length(locationmark(:,1));
end
locationmark(:,1)=[];
