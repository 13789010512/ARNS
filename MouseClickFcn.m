  function MouseClickFcn(src,event)
    pt=get(gca,'CurrentPoint');     %在当前坐标轴中获取鼠标点击的坐标位置
    location_mark(1,1)=pt(1,1);
    location_mark(1,2)=pt(1,2);
    if strcmp(get(gcf,'SelectionType'),'normal')
%        plot(location_juction(1,1),location_juction(1,2),'.', 'MarkerSize', 15, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g'); 
%        fdata1 = fopen('location_juctionMapofHighway.txt','a+');%
%        fprintf(fdata1,'%d ',location_juction);
%        fprintf(fdata1,'\n');
%        fclose(fdata1);
%        fdata1 = fopen('location_markMapofHighway.txt','a+');%
%        fprintf(fdata1,'%d ',location_juction);
%        fprintf(fdata1,'\n');
%        fclose(fdata1);
%      elseif strcmp(get(gcf,'SelectionType'),'alt')
%             plot(location_juction(1,1),location_juction(1,2),'.', 'MarkerSize', 15, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'g'); 
%             fdata1 = fopen('location_markMapofHighway.txt','a+');%
%             fprintf(fdata1,'%d ',location_juction);
%             fprintf(fdata1,'\n');
%             fclose(fdata1);
% %        elseif strcmp(get(gcf,'SelectionType'),'alt')
            plot(location_mark(1,1),location_mark(1,2),'.', 'MarkerSize', 15, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'g'); 
            fdata1 = fopen('D:\matlab2018a\result for all\system_branch\16-11.txt','a+');%
            fprintf(fdata1,'%d ',location_mark);
            fprintf(fdata1,'\n');
            fclose(fdata1);
     end
