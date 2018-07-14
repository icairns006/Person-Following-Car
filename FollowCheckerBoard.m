
%main code
function main
load cameraParams
cam = webcam('FaceTime HD Camera');
s=serial_init;
thr=1500;
turn=1500;


% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
runLoop = true;
numPts = 0;
frameCount = 0;
Xcenter=frameSize(2)/2;
Ycenter=frameSize(1)/2;

while runLoop && frameCount < 200
    frameCount = frameCount + 1;
    
    % Read in image into an array.
    [rgbImage storedColorMap] = snapshot(cam);
    [rows columns numberOfColorBands] = size(rgbImage);
    videoFrame = rgbImage;
    
    [imagePoints,boardSize] = detectCheckerboardPoints(rgbImage);
    S=std(imagePoints); 
    if size(imagePoints)>0 & S(1)>10 & S(2)>10 & max(boardSize)==10 & min(boardSize)==7
        videoFrame = insertMarker(videoFrame, imagePoints, '+', 'Color', 'red');
        centroids=mean(imagePoints);
        centroidsX = centroids(1);
        centroidsY = centroids(2);
        Xdelta=centroidsX-Xcenter;
        Ydelta=centroidsY-Ycenter;

        thr=1500-Ydelta;
        turn=1500+Xdelta*sign(Ydelta);
        ArdWrite(s,thr,turn);
        videoFrame = insertMarker(videoFrame, [centroidsX, centroidsY], '+', 'Color', 'red','size',10);

     
    else
        ArdWrite(s,1500,1500);
        
    end
        
    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
    


end

ArdWrite(s,1500,1500);
clear cam;
serial_close(s)

end

function y=ArdWrite(s,thr,turn)
    if thr>1600
        thr=1600;
    elseif thr<1300
        thr=1300;
    end
    if turn>2000
        turn=2000;
    elseif turn<1000
        turn=1000;
    end

    thrSTR=num2str(thr);
    turnSTR=num2str(turn);
    strout = strcat(thrSTR,',',turnSTR)
    fprintf(s,strout);
end

%set up serial
function s=serial_init
    s = serial('/dev/tty.usbmodem1421'); % Be certain to know which port you are connected to.
    set(s,'BaudRate',9600);
    fopen(s);
    %del
    for i=1:150
        pause(.01);
    end
end

function serial_close(s)
    fclose(s);
    delete(s);
    clear s
end