writerObj = VideoWriter('myVideo.avi');
  writerObj.FrameRate = 10;
  % set the seconds per image
% open the video writer
open(writerObj);
for i=91:208
    % convert the image to a frame
    image=sprintf( '%s/image%04d.fig', 'anj_result', i );
    frame =  openfig(image);    
    writeVideo(writerObj, frame);
end
close(writerObj);