# OpenCV - Python

Author: Fasermaler



This documentation serves to collate various OpenCV concepts I have had to work with into easy-to-use template code.

This will not be a carbon copy of the [OpenCV documentation](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_tutorials.html) because it would not make sense to lift the official docs when referencing the official docs is faster and more comprehensive. 

## Imports

### Basic Imports

Basic imports needed for OpenCV to even work.

#### cv2

```python
import cv2
```

Imports OpenCV. On older code, you might have to import it as `cv` instead and even then there might be a few changes needed as some attribute names were modified.

```python
import cv2 as cv
```

#### numpy

```python
import numpy as np
```

`np` is simply the convention for most example code. If the code does not explicitly require `numpy`, this import statement can be omitted without significant issues.

### Other Useful Imports

Here are some other additional imports that could be useful. Do note that these will have to be installed in the virtual environment or just `pip` installed depending on the circumstance.

#### glob

```python
import glob
```

`glob` is a very useful in-built python library when trying to process multiple images or videos. It allows the program to list files within a directory (even by extension), which then allows the program to `cv2.imread` the files sequentially for processing. Glob documentation can be found: [here](https://docs.python.org/3/library/glob.html).

#### Pillow

```python
import PIL
```

Or sometimes:

```python
import PIL as Image
```

`PIL` or the `Pillow` library is a powerful image processing python library that adds additional tools to the script in modifying images. These include resizing, rotation, histograms, automatic contrast enhancements and more. Full documentation: [here](https://pillow.readthedocs.io/en/stable/handbook/overview.html).

#### imutils

```python
import imutils
```

`imutils` or `image utilities` is a convenience library that contains various functions such as skeletonization, translation, fps counter and etc. It's most important use is when referring to sample code by [pyimagesearch](https://www.pyimagesearch.com/) as their code sees judicious use of the `imutils` library. Documentation for `imutils` can be found: [here](https://github.com/jrosebr1/imutils).

#### Time

```python
import time
```

`time` is a very useful library overall but especially when dealing with image post-processing work. It can be used to generate the evaluation time per frame as well as used as the file name for output images.

*Tip: If the script requires an FPS counter, you can simply use `imutils` instead of making one on your own using the `time` library. It will help you to save... time.*

## Image I/O

### Image Read

Basic image read function.

```python
image = cv2.imread('/path/to/image.jpg')
```

Accepts both relative and absolute paths - though absolute paths is recommended especially when dealing with neural networks and the like.

It is also possible to set flags when calling `.imread()`

```python
image = cv2.imread('/path/to/image.jpg', CV_LOAD_IMAGE_COLOR) # Convert image to color
image = cv2.imread('/path/to/image.jpg', CV_LOAD_IMAGE_GRAYSCALE) # Convert image to grascale
```

### Image Write

Basic image write function.

```python
cv2.imwrite('/path/to/image.jpg', frame)
```

This writes the pixel data stored in `frame` as an image saved on the HDD at the path `/path/to/image.jpg`.

It is possible to set certain parameters when calling `.imwrite()`

```python
cv2.imwrite('/path/to/image.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90]) # Set JPEG quality to 90

cv2.imwrite('/path/to/image.png', frame, [CV_IMWRITE_PNG_COMPRESSION, 9]) # Set PNG compression level to 9 

```

### Supported image extensions

The following is a list of supported file extensions for images for both `.imread()` and `.imwrite()`.

- Bitmaps: `*bmp`, `*.dib`
- JPEG: `*.jpeg`, `*.jpg`, `*.jpe`
- JPEG 2000: `*.jp2`
- Portable Network Graphics: `*.png`
- WebP: `*.webp`
- Portable image format: `*.pbm`, `*.pgm`, *.ppm`
- Sun Rasters: `*.sr`, `*.ras*
- TIFF: `*.tiff`, `*.tif`

Do note that some file extensions might look for codecs within the OS image. It might be necessary to install the necessary packages for full support (though JPEG and PNG are always supported).

## Video I/O

### Webcam / USB Connected Camera

To get footage from a webcam or USB connected camera, it is necessary to first define the capture object (the convention is to call it `cap`). Then invoke the `.read()` subroutine on the capture object to yield 1 frame.

```python
cap = cv2.VideoCapture(0) # Set the capture object

ret, frame = cap.read() # Returns a frame

cap.release() # Release the capture object
```

Firstly, the `VideoCapture` object is set to `0`. By default, the first webcam connected to the computer will be `0`. The number can be changed if there are multiple webcams and a specific one has to be connected to. Additionally, if a script connected to a camera fails to exit properly or release the capture object properly, you might find that the capture object number has to be incremented on the subsequent runs. Due to this, ensure that scripts always exit properly and release the capture object before doing so.

`.read()` returns a return, `ret` as well as the frame, `frame`. `ret` and `frame` are merely common conventions.

### IP Camera

It is also possible to connect to an IP camera as OpenCV supports FFMPEG via HTTP or RTSP. Simply feed the relevant URL into the `VideoCapture` object. [ispyconnect](https://www.ispyconnect.com/) has a good list of camera URLs and their open source utility can be used to find the appropriate URL of the camera as well.

#### Process

1. Check that the IP address assigned to the camera is within the same range of the camera. For instance, if the camera is set to `192.168.0.11`, the computer IP address should be set to `192.168.0.xx`. This can be checked using `ipconfig` in the command prompt (windows) or `ifconfig` in the bash terminal (linux). 

   - If the IP addresses are not within the same range, the camera cannot be connected to. Assign a static IP to the computer under the network settings

2. Login to the camera normally using the manufacturer's camera viewing utility to verify the camera is operational and that the login details are correct.

   - Alternatively, connect to the camera using the ispyconnect utility

3. If everything is in order, find the correct URL - it should look something like this:

   ```python
   http://admin:12345@192.168.0.11/?cgi-bin/h264
   ```

Simply feed that exact URL into the `VideoCapture` object:

```python
cap = cv2.VideoCapture(http://admin:12345@192.168.0.11/?cgi-bin/h264) # Set the capture object

# Rest of the code is identical
ret, frame = cap.read() 

cap.release() 
```

#### Troubleshooting Notes

If the URL does not seem to work, do not worry - as long as the URL works on browser or in ispyconnect, the actual URL cannot be far off.

1. For RTSP, generally only the login details and IP are required. So if there are extra strings behind the URL, try removing them to see if it works.

   ```python
   cap = cv2.VideoCapture(http://admin:12345@192.168.0.11/MediaInput) # This might not be working
   
   cap = cv2.VideoCapture(http://admin:12345@192.168.0.11) # But this might work
   ```

2. Check that the port is correct - sometimes the URL will require a port. Remove the port if there is one and try to find a port if there isn't one.
3. Double check the URL - ensure that you are not using HTTPS or that the RTSP link does not start with a HTTP header by mistake.

### Full Video Display Script

In most cases, we do not only want 1 frame but we seek to grab multiple frames from the camera. This section will show the full example script.

```python
import cv2

cap = cv2.VideoCapture(0) # Set the capture object

while True:
	ret, frame = cap.read() # Returns a frame
    
    if ret:
        cv2.imshow('frame', frame)
        
        # break is 'Q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        # break if no frames left in the video
        break

cv2.destroyAllWindows() # Destroy the viewing window
cap.release() # Release the capture object
```

### Video Writing

To write video, first define the output object with the relevant codec:

```python
img_width = 640
img_height = 480
video_fps = 30

out = cv2.VideoWriter('/path/to/output.avi', cv2.VideoWriter_fourcc(*'XVID'), video_fps, (img_width, img_height)) # defines output object for .avi output format

out = cv2.VideoWriter('/path/to/output.mp4', cv2.VideoWriter_fourcc(*'MPEG'), video_fps, (img_width, img_height)) # defines output object for .mp4 output format
```

When defining the output writer, the resolution and codec are very important. Ensure that the codec matches the video format to be saved. If the resolution of the writer object does not match the frame size the video file will also be corrupted.

If the video FPS does not match up, the output video might be faster or slower though it does not result in file corruption.

Whenever a new frame is obtained, simply write the frame to the video writer:

```python
out.write(frame)
```

Then close the output writer at the end of the program. If the output writer is not closed properly, the video file will also be corrupted.

```python
out.release()
```

## Image Related Functions

### Shape

The shape attribute returns the rows, columns and channels of the image (in that order).

```python
img_height = image.shape[0]
img_length = image.shape[1]
img_layers = image.shape[2]
```

`.shape[0]` and `.shape[1]` are some of the most commonly used attributes. It is easy to confuse the two so do refer back when troubleshooting code.

### Slicing Images

It is possible to slice images by working on them as numpy arrays. This would allow a specific section of the image to be cropped and saved as another image.

```python
crop = frame[y_start:y_end, x_start:x_end]
```

A more concrete example can be found in the `split_four.py` script that can be found in the example code folder.

```python
def split_into_four(frame):
		
		Width, Height = frame.shape[1], frame.shape[0]	
		crop_h, crop_w = int(Height/2), int(Width/2)
 		
 		# Get the 4 cropped frames and put them in an array
		crop1 = frame[0:crop_h, 0:crop_w]
		crop2 = frame[0:crop_h, crop_w:Width]
		crop3 = frame[crop_h:Height, 0:crop_w]
		crop4 = frame[crop_h:Height, crop_w:Width]
		crop_array = [crop1, crop2, crop3, crop4]

		return crop_array
```

In this example, the image is split into 4 equally sized images. The way this is done is by finding the halfway point of both the height and width of the frame (assigned to `crop_h` and `crop_y`).

The script then proceeds to slice the image 4 times to form 4 cropped images which are then placed within a crop array to be returned by the function.

## Keystrokes

OpenCV uses the `waitKey` function to draw new frames on to any OpenCV active windows. This means that the `waitKey` function has 2 secondary purposes

- Primary Function: Update the window 
- Secondary Function 1: Allows developer to control the delay of the update loop
- Secondary Function 2: Allows developer to check for specific keystrokes to do specific actions

### Close Program

The most ubiquitous use of all is the use of the `waitKey` function to close a program.

In most OpenCV programs (especially pertaining to video), there is an active loop within the program. Should this loop be handled or exited incorrectly, it can cause various memory or GPU issues. 

This is why the `waitKey` function is often used to check if the quit key (usually assigned to `q` by convention) has been pressed.

```python
while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('frame', frame)
    
    # if 'q' was pressed, break out of the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
cap.release()
cv2.destroyAllWindows()
```

### Adding Additional User Functionality with Keystrokes

Most people have taken to appending `if cv2.waitKey(1) & 0xFF == ord('q'):` automatically to the end of their loops. But I personally think that the next approach, while more verbose is superior. It is more extensible and builds a better habit than appending a line without thinking about what it actually means.

In the case where you'd like to add a functionality to `c`apture an image in a video stream, the `waitKey` function can also be used to help detect the `c` keystroke just like it was used to detect the `q` keystroke as a break condition.

```python
key = cv2.waitKey(1) & 0xFF

if key == ord("q"):
    break
elif key == ord('c'):
	cv2.imwrite('output.jpg', image)
```

#### Working with Special Keys

In the event that a special key is desired such as `Esc`, `spacebar` and etc. The dec code of the key can be used instead. Refer to the [ASCII table](http://www.asciitable.com/) for specific dec codes.

The following is a snippet of code from an ROI selection utility I wrote in the past. In this case, the `Esc` key is used to allow the user to reset the currently selected ROI points.

```python
# Various key events
key = cv2.waitKey(1) & 0xFF

# Esc key will reset the ref_points list
if key == 27: # 27 is the dec value for the Escape Character
    ref_points = []
    status = "RESET"

# U key will remove last point selected
if key == ord('u'):
    ref_points.pop()
    status = "UNDO"
    
# S key will save if the ref_points list has 4 points
if key == ord('s'):
	if len(ref_points) < 4:
		status = "ERROR"
	else:
        cfg_read.write_roi_coordinates(*corrected_ref_points)
        status = "SAVED"
        
# Q will exit
if key == ord('q'):
	break
```



## Working with Picamera

The RPi Picamera is unique as it connects directly to the RPi using a ribbon cable. Due too this it requires use of the `picamera` library (to be installed separately) and does not use the `VideoCapture` object.

### Imports

`picamera` generally requires 2 imports in order to work.

```python
from picamera.array import PiRGBArray
from picamera import PiCamera
```

### Initialization

Basic initialization of the Pi Camera is as follows:

```python
resolution_height = 240
resolution_width = 426

camera = PiCamera()
camera.resolution = (resolution_width, resolution_height)
camera.framerate = 24
rawCapture = PiRGBArray(camera, size=(resolution_width, resolution_height))
```

Check that the resolution is correct. Most Pi Cameras support a few very specific resolutions and associated frame-rates. The `picamera` library will warn and default to the closest available resolution if the input resolution is invalid. However, this might break other parts of the code so do be aware of this.

#### Additional Parameters

One of the more powerful aspects of using a Pi Camera is the ability to modify the camera settings on the software side instead of going into a camera menu built into the camera firmware. This allows the camera lighting and other related settings to be set to specifically be optimal for a set of lighting conditions, boosting performance in specific use cases.

Some other useful parameters include the following:

```python
camera.exposure_mode = 'off' # Turn off auto exposure compensation
camera.exposure_compensation = -3 # Set the specific level of exposure compensation
camera.drc_strength = 'off' # Sets the dynamic range compression
camera.still_stats = False # Retrieves or sets whether statistics will be calculated from still frames or the prior preview frame
camera.awb_mode = 'off' # Turn off auto white balance
camera.awb_gains = (Fraction(25, 16), Fraction(25,16)) # Set a specific white balance gain
```

The full API (and associate list of options) can be found: [here](https://picamera.readthedocs.io/en/release-1.9/api.html). 

##### Setting White Balance

With regards to `camera.awb_gains`, it is better to print out the `awb` value first by invoking the camera attribute:

```python
print(camera.awb_gains)
```

Then compensate accordingly. The `.awb_gains` attribute requires the use of the `fraction` library so the full setting should be as follows:

```python
from fractions import Fraction

camera.awb_mode = 'off' # Turn off auto white balance

camera.awb_gains = (Fraction(25, 16), Fraction(25,16)) # Set a specific white balance gain
```

### Reading Video from Pi Camera

Reading video from the Pi Camera involves reading from the camera `rawCapture` buffer. This means that the entire capture object method normally used is completely circumvented. The following is the code for reading frames after the Pi Camera has been initialized:

```python
for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    frame = img.array
    
    rawCapture.truncate(0)
```

It is important to truncate the `rawCapture` buffer or else it will not receive any more frames. 

Now with OpenCV viewing functionality:

```python
for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    frame = img.array
    cv2.imshow('frame', frame)
    
    rawCapture.truncate(0)
	if k == 0xFF & ord("q"):
        break
        
cv2.destroyAllWindows()
camera.close()
```

#### Video Formats

In addition to the `bgr` format, `picamera` also supports other formats.

- `jpeg`
- `png`
- `gif`
- `bmp`
- `yuv`
- `rgb`
- `rgba`
- `bgr`
- `bgra`

However as we are working with OpenCV, `bgr` will likely be the format used.

#### Flushing the Frame Buffer

Occasionally in certain use cases, the output framer-ate would significantly stutter, dropping to as low as 2 FPS where one would expect 24 or 30 FPS. On possible solution is to simply flush the frame buffer. In most cases, there will be no noticeable hit to performance if the actual camera frame-rate is high (24 or higher) and the buffer is set low (5 or so frames).

```python
for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    for i in range(5): # Clears the 5 frame buffer 
        frame = img.array
        
    frame = img.array
    
    rawCapture.truncate(0)
```

### Scripts

The following are the scripts available in the sample code/template code folder:

- `pi_camera_single_frame.py`: Grabs a single pi camera frame and displays it. Can be used to test if the pi camera even works
- `pi_camera_video.py`: Grabs video output from the pi camera. Has various options for the camera as well as the option to flush the frame buffer. Also allows you to capture the active frame by pressing `c`.

## Miscellaneous Tricks

Some miscellaneous functions and tricks that I use.

### Limit Display FPS 

This allows the display loop to be set at a specific desired FPS.

```python
desired_FPS = 30

img_ms = int(1000 / desired_FPS)
if cv2.waitKey(img_ms) & 0xFF == ord('q'):
    break
```

### Use Odd Shaped ROI

The use case is as follows: 

- The ROI is made up of a bunch of points that do not form a nice rectangle
- The script has to reject bounding boxes beyond the ROI

The approach uses contours and compares the center of bounding boxes to the contour.

```python
roi_coordinates = [(54, 500), (64, 3), (995, 678)]
ctr = np.array(cfg_read.roi_coordinates).reshape((-1,1,2)).astype(np.int32) # Creates a contour based on the ROI coordinates

cv2.drawContours(frame, [ctr], 0, (0, 255, 255),1) # Draws the contour on the frame (optional)
```

To compare a specific bounding box with format `[x, y, w, h]`

```python
x = bbox[0]
w = bbox[2]
y = bbox[1]
h = bbox[3]

dist = cv2.pointPolygonTest(ctr, (int(x + w/2), int(y + h/2)), True)

if dist >= 0:
	# If the bounding box is within the ROI
    # Draw the box as a green rectangle
    cv2.rectangle(frame, (x, y), ((x + w), (y + h)), (0,255,0), 3)
else:
    # If the bounding box is not within the ROI
    # Draw the box as a red rectangle
    cv2.rectangle(frame, (x, y), ((x + w), (y + h)), (0,0,255),
```

### Red Color Thresholding Using RGB

This is a method of color filtering that does not use hue and saturation, instead opting to work directly on the image layers using numpy.

The use case is highly specific and the filtering method itself works extremely well within a controlled condition but is not robust enough to adapt itself to changing lighting or situations. Thus it is not recommended in most cases - it is however, very accurate when placed within the specific set of circumstances it has been calibrated for.

To illustrate the legitimacy of this algorithm, here is an image of a drone flying into a red-lit hoop running on this filtering method:

![1563808324844](assets/1563808324844.png)

To reaffirm, this is a method used when detecting Red (or the other 2 base colors, Blue and Green). It will give a strong signal when used to evaluate base colors but is unlikely to work well with any other colors.

```python
# Sets the percentage red threshold, below which a pixel will be eliminated from the final result
PERC_RED_THRESH = 0.3

ret, frame = cap.read()

# Gets the individual color channels
# The format assumed here is bgr
b_channel = np.array(frame[:,:,0]).astype('float')
g_channel = np.array(frame[:,:,1]).astype('float')
r_channel = np.array(frame[:,:,2]).astype('float')

# Create a base channel where all values are added together
bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)

# Take the average of the blue and green channels and subtract it from the red channel
# This will eliminate white color from the final image
reduce_white = np.subtract(r_channel,((b_channel + g_channel)/ 2))

# Find the percentage red by dividing the previous channel by 255
perc_red = np.divide(reduce_white,255)

# Eliminate all pixels that fail to meet the minimum percentage red using numpy conditional array value setting
perc_red[perc_red < PERC_RED_THRESH] = 0

# Set the values back to within a range of 255 and set the type back to uint8 so that the frame becomes valid for OpenCV processing
final_image = perc_red * 255
final_image = np.floor(final_image).astype('uint8')

# Run another round of openCV thresholding as required
ret, th = cv2.threshold(final_image,10,255,cv2.THRESH_BINARY)
```

If you would like to try this approach for your use case, feel free to run the script `red_thresholding_demo.py` in the example code. 

**Fair Warning: Much tuning is required to produce good results**