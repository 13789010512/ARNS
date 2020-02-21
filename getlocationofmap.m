%%% 生成十字路口及折点位置信息，保存在txt文件中
clear;clc;
% delete('location_juction.txt');
% delete('location_mark.txt');
ima=imread('MapofHighway.jpg');
% % Y=edge(X,'canny');
% k=2;
% [mu,mask]=kmeans(ima,k);
% si=[2 2];
% sigma=0.99;
% thresh=0.99;

    imshow(ima);                  %显示图片

    hold on;
    set(gcf,'WindowButtonDownFcn',@MouseClickFcn);
