
%main code
function main
s=serial_init;
thr=1500;
turn=1500;


% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam('FaceTime HD Camera');

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

    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;

    if numPts < 10
        % Detection mode.
        bbox = faceDetector.step(videoFrameGray);

        if ~isempty(bbox)
            
            % Find corner points inside the detected region.
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));

            % Re-initialize the point tracker.
            xyPoints = points.Location;
            numPts = size(xyPoints,1);
            release(pointTracker);
            initialize(pointTracker, xyPoints, videoFrameGray);

            % Save a copy of the points.
            oldPoints = xyPoints;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints = bbox2points(bbox(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
            
            
        end

    else
       
        % Tracking mode.
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);

        numPts = size(visiblePoints, 1);

        if numPts >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
                oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints = transformPointsForward(xform, bboxPoints);

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');

            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);
            
            %find center of face in video
            Xcurrent=(bboxPolygon(1)+bboxPolygon(3)+bboxPolygon(5)+bboxPolygon(7))/4;
            Ycurrent=(bboxPolygon(2)+bboxPolygon(4)+bboxPolygon(6)+bboxPolygon(8))/4;
            
            Xdelta=Xcurrent-Xcenter;
            Ydelta=Ycurrent-Ycenter;
            
            thr=1500-Ydelta;
            turn=1500+Xdelta*sign(Ydelta);
            ArdWrite(s,thr,turn);
            
             
        end

    end
    
    

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(faceDetector);




%ArdWrite(s,thr,turn);


serial_close(s)

end

function y=ArdWrite(s,thr,turn)
    if thr>1640
        thr=1640;
    elseif thr<1380
        thr=1380;
    end
    if turn>2000
        turn=2000;
    elseif turn<1000
        turn=1000;
    end

    thrSTR=num2str(thr);
    turnSTR=num2str(turn);
    strout = strcat(thrSTR,',',turnSTR);
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