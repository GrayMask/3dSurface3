function readVideo( )
fileName = 'v.MOV'; 
obj = VideoReader(fileName);
numFrames = 2 * 120;
numFrames2 = 5 * 120;
a=[];
 for k = numFrames : numFrames2
     frame = read(obj,k);
     imwrite(frame,strcat(num2str(k),'.jpg'),'jpg');
     a(end+1) = mean(frame(:));
 end

 x=1:1:numFrames;
 x = x/30;
 figure(x ,a);
end

function figure(a ,b)
plot(a,b);
%axis([0 3 5 55])
%set(gca,'XTick',1:1:3) %¸Ä±äxÖá×ø±?ä¸ôÏÔÊ¾ ÕâÀ?ä¸ôÎª10
%set(gca,'YTick') %¸Ä±äyÖá×ø±?ä¸ôÏÔÊ¾ ÕâÀ?ä¸ôÎª10
title('The Mean Pixel Values of First 3 Seconds in a Video')
xlabel('Time (s)');
ylabel('Mean of Pixel Value');
end

