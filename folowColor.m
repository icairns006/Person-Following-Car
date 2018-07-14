
%main code
function main
cam = webcam('FaceTime HD Camera');
%s=serial_init;
thr=1500;
turn=1500;
% Assign the low and high thresholds for each color band.
        redThresholdLow = 0;
		redThresholdHigh = 120;
		greenThresholdLow = 100;
		greenThresholdHigh = 255;
		blueThresholdLow = 0;
		blueThresholdHigh = 40;


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
    
    
    if strcmpi(class(rgbImage), 'uint8')
		% Flag for 256 gray levels.
		eightBit = true;
	else
		eightBit = false;
    end
    % Extract out the color bands from the original image
	% into 3 separate 2D arrays, one for each color component.
    redBand = rgbImage(:, :, 1); 
	greenBand = rgbImage(:, :, 2); 
	blueBand = rgbImage(:, :, 3); 
    
    % Compute the red histogram. 
    [countsR, grayLevelsR] = imhist(redBand); 
	maxGLValueR = find(countsR > 0, 1, 'last'); 
	maxCountR = max(countsR); 
    % Compute and plot the green histogram. 
    [countsG, grayLevelsG] = imhist(greenBand); 
	maxGLValueG = find(countsG > 0, 1, 'last'); 
	maxCountG = max(countsG); 
    % Compute and plot the blue histogram. 
    [countsB, grayLevelsB] = imhist(blueBand); 
	maxGLValueB = find(countsB > 0, 1, 'last'); 
	maxCountB = max(countsB); 
    
    
    maxGL = max([maxGLValueR,  maxGLValueG, maxGLValueB]); 
	if eightBit 
			maxGL = 255; 
	end 
	maxCount = max([maxCountR,  maxCountG, maxCountB]); 
    
    maxGrayLevel = max([maxGLValueR, maxGLValueG, maxGLValueB]); 
    
    if eightBit 
		xlim([0 255]); 
	else 
		xlim([0 maxGrayLevel]); 
    end 
    
    if eightBit
			redThresholdLow = uint8(redThresholdLow * 255);
			greenThresholdHigh = uint8(greenThresholdHigh * 255);
			blueThresholdHigh = uint8(blueThresholdHigh * 255);
    end
    
    % Now apply each color band's particular thresholds to the color band
    redMask = (redBand >= redThresholdLow) & (redBand <= redThresholdHigh);
	greenMask = (greenBand >= greenThresholdLow) & (greenBand <= greenThresholdHigh);
	blueMask = (blueBand >= blueThresholdLow) & (blueBand <= blueThresholdHigh);
    
    % Combine the masks to find where all 3 are "true."
	% Then we will have the mask of only the red parts of the image.
	redObjectsMask = uint8(redMask & greenMask & blueMask);
    
    smallestAcceptableArea = 600; % Keep areas only if they're bigger than this.
    
    % Get rid of small objects.  Note: bwareaopen returns a logical.
	redObjectsMask = uint8(bwareaopen(redObjectsMask, smallestAcceptableArea));
    
    
    % Smooth the border using a morphological closing operation, imclose().
	structuringElement = strel('disk', 4);
	redObjectsMask = imclose(redObjectsMask, structuringElement);
    
    % Fill in any holes in the regions, since they are most likely red also.
	redObjectsMask = uint8(imfill(redObjectsMask, 'holes'));
    
    % You can only multiply integers if they are of the same type.
	% (redObjectsMask is a logical array.)
	% We need to convert the type of redObjectsMask to the same data type as redBand.
	redObjectsMask = cast(redObjectsMask, class(redBand)); 
    
    % Use the red object mask to mask out the red-only portions of the rgb image.
	maskedImageR = redObjectsMask .* redBand;
	maskedImageG = redObjectsMask .* greenBand;
	maskedImageB = redObjectsMask .* blueBand;
    
    % Concatenate the masked color bands to form the rgb image.
	maskedRGBImage = cat(3, maskedImageR, maskedImageG, maskedImageB);
    
    % Measure the mean RGB and area of all the detected blobs.
	
        bw = rgb2gray(maskedRGBImage);
        labeledImage = bwlabel(bw, 8);
        blobMeasurements = regionprops(labeledImage, bw, 'all');
        if size(blobMeasurements)>0
            allAreas = [blobMeasurements.Area];
            [sortedAreas, sortIndexes] = sort(allAreas, 'descend');
            if size(sortIndexes)>0
                biggestBlob = ismember(labeledImage, sortIndexes(1));
                bob=regionprops(biggestBlob, bw, 'all');
                blobarea=[bob.Area];


               % allBlobCentroids = [blobMeasurements.Centroid];
                %[len, wid]=size(allBlobCentroids);


                if blobarea>2000
                    centroids = bob.Centroid;
                    centroidsX = centroids(1);
                    centroidsY = centroids(2);
                    Xdelta=centroidsX-Xcenter;
                    Ydelta=centroidsY-Ycenter;

                    thr=1500-Ydelta;
                    turn=1500+Xdelta*sign(Ydelta);
                    %ArdWrite(s,thr,turn);
                    videoFrame = insertMarker(videoFrame, [centroidsX, centroidsY], '+', 'Color', 'red','size',30);
                end
            end
        end
        
    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
    


end


clear cam;
%serial_close(s)

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