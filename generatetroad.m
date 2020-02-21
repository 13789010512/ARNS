function generatetroad(location_mark)% ,location_jucntion,location_branch)
%%%生成道路像素信息，及表示出道路轮廓及十字路口
ima=imread('MapofHighway.jpg');
 sizeima=size(ima);
 sizey=sizeima(2);
 imshow(ima);                  %显示图片
 hold on;
 %���ƹ���·��
for i=1:length(location_mark(:,1))-1
    A=location_mark(i,1:2);
    B=location_mark(i+1,1:2);
    plot([A(1),B(1)],[A(2),B(2)],'Linewidth',3,'Color',[34/255,139/255,34/255]);
%     plot(A(1),A(2),'+', 'MarkerSize', 12, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'g','Linewidth',2); 
    hold on
end
%  %���Ʒֲ�·��
% for i=1:length(location_branch(:,1))
%     A=location_branch(i,1:2);
%     if location_branch(i,3)~=0
%         B=location_jucntion(location_branch(i,3),2:3);
%     else B=location_branch(i-1,1:2);
%     end
%     plot([A(1),B(1)],[A(2),B(2)],'Linewidth',3,'Color',[34/255,139/255,34/255]);
% end
%  %����ʮ��·��
% for i=1:length(location_jucntion(:,1))
%      plot(location_jucntion(i,2),location_jucntion(i,3),'+', 'MarkerSize', 12, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'g','Linewidth',2); 
% end
%���ƿ̶ȳ�
Linmap400m=400*sum((location_mark(5,1:2)-location_mark(6,1:2)).^2).^0.5/287;

 plot([20,20+Linmap400m],[80,80],'Linewidth',2);
 text(30+Linmap400m/20,65,'400m')


