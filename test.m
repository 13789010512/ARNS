clear;clc;delete('D:\matlab2018a\result for all\system_branch\16-11.txt');
ima=imread('system_map_bar.jpg');
% % Y=edge(X,'canny');
% k=2;
% [mu,mask]=kmeans(ima,k);
% si=[2 2];
% sigma=0.99;
% thresh=0.99;

    imshow(ima);                  %œ‘ æÕº∆¨

    hold on;
    set(gcf,'WindowButtonDownFcn',@MouseClickFcn);
