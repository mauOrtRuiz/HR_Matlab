% Open video
v = VideoReader("Video_MOR.mp4");
frame = readFrame(v);
fps=24;

% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Capture one frame to get its size.
frameSize = size(frame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [200 200 [frameSize(2), frameSize(1)]+30]);

% Preparation to open full video
runLoop = true;
numPts = 0;
frameCount = 0;
ExtraPoints=4;
a = 5;
b = 20;
xr = round((b-a).*rand(ExtraPoints,1) + a);
yr = round((b-a).*rand(ExtraPoints,1) + a);
[xr yr]
SizeX=frameSize(1);
SizeY=frameSize(2);
tser=1;

while runLoop && frameCount < 200
    % Get the next frame.
    videoFrame = readFrame(v);
    imagenRGB=videoFrame;
    MaskDinam=createMaskFilter(imagenRGB);
    BW2_big = bwareafilt(MaskDinam,5);
    videoHSV=rgb2hsv(videoFrame);
    videoFrameGray = videoFrame(:,:,2);
    frameCount = frameCount + 1;
    if numPts < 10
        % Detection mode.
        bbox=step(faceDetector, videoFrame);
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
            xyPoints(1,:)
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
            [xform, inlierIdx] = estgeotform2d(...
                oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);
            oldInliers    = oldInliers(inlierIdx, :);
            visiblePoints = visiblePoints(inlierIdx, :);
            
            % Apply the transformation to the bounding box.
            bboxPoints = transformPointsForward(xform, bboxPoints);
            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);
            
            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
            
            
            VecPoi=round(visiblePoints);
            kkcon=1;
            ToT=height(VecPoi);

            % In this loop only skin points are detected and selected
            for kkk=1:ToT
                VectorValid(kkk,tser)=0;
                Vec2=round(visiblePoints(kkk,:));
                PixelInd=imagenRGB(VecPoi(kkk,2),VecPoi(kkk,1),:);
                VSig(kkk,tser)=double(videoFrameGray(Vec2(2),Vec2(1)))/255;
                if createMaskFilter(PixelInd)
                  
                  % Rows are points, columns are each video frame 
                  VectorValid(kkk,tser)=1;
                  NewvisiblePoints(kkcon,:)=visiblePoints(kkk,:);
                  kkcon=kkcon+1;
                  
                  
                  
                end
            end
            
             % Display tracked points.
            videoFrame= insertMarker(videoFrame,NewvisiblePoints,"+");
            
           
            
            
            
            % Vector size:  features.NumFeatures X Points_current.Count
          
            tser=tser+1;
            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);
        end
    end
    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end


% extract signal, only signal from all valis skin points are extracted
%
for k=1:216
    V(k)=sum(VectorValid(k,:));
end
cont2=1;
for k=1:216
    if(V(k)==199)
     figure
     X(cont2,:)=VSig(k,:);
     cont2=cont2+1;
     plot(VSig(k,:))
    end
end

% As a sample we extract file #18
% Raw data from green channel is plotted
X8=X(18,:);
figure(5)
plot(X8)
grid
title('Raw data signal of skin point 8')


% Remove Trend
X8Det=detrend(X8);
figure(6)
plot(X8Det)
grid
title('Detrented signal point 8')

% Smooth filter
Par = 3;
MediaFilter = ones(1, Par)/Par;
avgX8 = filter(MediaFilter, 1, X8Det);
figure(7)
T=length(avgX8)
nn=1:199;
tim=nn*(1/24);
plot(tim,avgX8)
grid
title('Average filter')


% PSD is computed
figure(7)
x=avgX8;
fs=fps;
N = length(x);
xdft = fft(x);
xdft = xdft(1:N/2+1);
psdx = (1/(fs*N)) * abs(xdft).^2;
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 60*(0:fs/length(x):fs/2);
%plot(freq,pow2db(psdx))
plot(freq,psdx)
grid on
title("Periodogram Using FFT")
xlabel("Frequency (Hz)")
ylabel("Power/Frequency (dB/Hz)")
