# OpenCV - Python

Author: Fasermaler

This documentation serves to collate various OpenCV concepts I had to self-learn. I have also compiled some of them into easy-to-use template code.

This will not be a carbon copy of the [OpenCV documentation](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_tutorials.html) because it would not make sense to lift the official docs when referencing the official docs is faster and more comprehensive. Instead, it serves to bridge the holes in understanding and certain approaches I have found to work in the past.

I will also attempt to supplement the content with real problems and solutions I have worked on in the past where possible.

_______

## Table of Contents <a name="top"></a> 

1. [Imports](#1)<br/>
   1.1	[Basic Imports](#1.1)</br>
   1.1.1	[cv2](#1.1.1)</br>
   1.1.2	[numpy](#1.1.2)</br>
   1.2	[Other Useful Imports](#1.2)</br>
   1.2.1	[glob](#1.2.1)</br>
   1.2.2	[Pillow](#1.2.2)</br>
   1.2.3	[imutils](#1.2.3)</br>
   1.2.4	[Time](#1.2.4)</br>
2. [Image I/O](#2)</br>
   2.1	[Image Read](#2.1)</br>
   2.2	[Image Write](#2.2)</br>
   2.3	[Supported Image Extensions](#2.3)</br>
3. [Video I/O](#3)</br>
   3.1	[Webcam/USB Connected Camera](#3.1)</br>
   3.2	[IP Camera](#3.2)</br>
   3.2.1	[Process](#3.2.1)</br>
   3.2.2	[Troubleshooting Notes](#3.2.2)</br>
   3.3	[Full Video Display Script](#3.3)</br>
   3.4	[Video Writing](#3.4)</br>
4. [Image Related Functions](#4)</br>
   4.1	[Shape](#4.1)</br>
   4.2	[Slicing Images](#4.2)</br>
5. [Keystrokes](#5)</br>
   5.1	[Close Program](#5.1)</br>
   5.2	[Adding Additional User Functionality with Keystrokes](#5.2)</br>
   5.2.1	[Working with Special Keys](#5.2.1)</br>
6. [Working with Picamera](#6)</br>
   6.1	[Imports](#6.1)</br>
   6.2	[Initialization](#6.2)</br>
   6.2.1	[Additional Paramerters](#6.2.1)</br>
   6.2.1.1 [Setting White Balance](#6.2.1.1)</br>
   6.3	[Reading from Pi Camera](#6.3)</br>
   6.3.1	[Video Formats](#6.3.1)</br>
   6.3.2	[Flushing the Frame Buffer](#6.3.2)</br>
   6.4	[Scripts](#6.4)</br>
7. [Drawing Functions](#7)</br>
   7.1	[Rectangle](#7.1)</br>
   7.2	[Text](#7.2)</br>
   7.2.1	[Font Options](#7.2.1)</br>
   7.2.2	[Common Text Locations](#7.2.2)</br>
   7.3	[Line](#7.3)</br>
   7.4	[Circle](#7.4)</br>
   7.5	[Other Drawing Functions](#7.5)</br>
8. [Hough Lines Transform](#8)</br>
   8.1	[Basic Hough Lines Transform Implementation](#8.1)</br>
9. [Hough Circles Transform](#9)</br>
   9.1	[Simple Demo](#9.1)</br>
10. [Morphology Transforms](#10)</br>
    10.1	[Preparing an Image for Morphology Transforms](#10.1)</br>
    10.1.1	[Convert to Grayscale](#10.1.1)</br>
    10.1.2	[Image Thresholding](#10.1.2)</br>
    10.1.3	[Kernel](#10.1.3)</br>
    10.1.4	[Inverting an Image](#10.1.4)</br>
    10.2	[Erode and Dilate](#10.2)</br>
    10.3	[Black Hat and Top Hat Transforms](#10.3)</br>
    10.4	[Demo Script](#10.4)</br>
11. [Centroid Detection](#11)</br>
    11.1	[Contour Detection](#11.1)</br>
    11.2	[Centroid Detection of Contours](#11.2)</br>
12. [Mouse Events](#12)</br>
    12.1	[Bind Callback Function](#12.1)</br>
    12.2	[Mouse Event Detection](#12.2)</br>
    12.3	[Scripts](#12.3)</br>
13. [SIFT](#13)</br>
    13.1	[SIFT in OpenCV 3](#13.1)</br>
    13.2	[Using SIFT](#13.2)</br>
    13.3	[When not to use SIFT](#13.3)</br>
14. [Camera Calibration](#14)</br>
    14.1	[Checkerboard Calibration](#14.1)</br>
    14.2	[Getting the Camera Mtx](14.2)</br>
    14.3	[Getting the Optimal Camera Matrix for Undistortion](#14.3)</br>
    14.4	[Steps for Demo](#14.4)</br>
15. [Aruco Marker Detection](#15)</br>
16. [Miscellaneous Tricks and Scripts](#misc)</br>
    16.1	[Limit Display FPS](#misc.1)</br>
    16.2	[Video Cropping Utility](#misc.2)</br>
    16.3	[ROI Selection Utility](#misc.3)</br>
    16.4	[Use Odd Shaped ROI](#misc.4)</br>
    16.5	[Working with Model Bounding Boxes](#misc.5)</br>
    16.6	[Circle center detection using Circumcenter](#misc.6)</br>
    16.7	[Arrow Detection with Hough Lines Transform](#misc.7)</br>
    16.8	[Red Color Thresholding Using RGB](#misc.8)</br>

## 1 Imports <a name="1"></a>

[go to top](#top)

### 1.1 Basic Imports <a name="1.1"></a>

[go to top](#top)

Basic imports needed for OpenCV to even work.

#### 1.1.1 cv2 <a name="1.1.1"></a>

[go to top](#top)

```python
import cv2
```

Imports OpenCV. On older code, you might have to import it as `cv` instead and even then there might be a few changes needed as some attribute names were modified.

```python
import cv2 as cv
```

#### 1.1.2 numpy <a name="1.1.2"></a>

[go to top](#top)

```python
import numpy as np
```

`np` is simply the convention for most example code. If the code does not explicitly require `numpy`, this import statement can be omitted without significant issues.

### 1.2 Other Useful Imports <a name="1.2"></a>

[go to top](#top)

Here are some other additional imports that could be useful. Do note that these will have to be installed in the virtual environment or just `pip` installed depending on the circumstance.

#### 1.2.1 glob <a name="1.2.1"></a>

[go to top](#top)

```python
import glob
```

`glob` is a very useful in-built python library when trying to process multiple images or videos. It allows the program to list files within a directory (even by extension), which then allows the program to `cv2.imread` the files sequentially for processing. Glob documentation can be found: [here](https://docs.python.org/3/library/glob.html).

#### 1.2.2 Pillow <a name="1.2.2"></a>

[go to top](#top)

```python
import PIL
```

Or sometimes:

```python
import PIL as Image
```

`PIL` or the `Pillow` library is a powerful image processing python library that adds additional tools to the script in modifying images. These include resizing, rotation, histograms, automatic contrast enhancements and more. Full documentation: [here](https://pillow.readthedocs.io/en/stable/handbook/overview.html).

#### 1.2.3 imutils <a name="1.2.3"></a>

[go to top](#top)

```python
import imutils
```

`imutils` or `image utilities` is a convenience library that contains various functions such as skeletonization, translation, fps counter and etc. It's most important use is when referring to sample code by [pyimagesearch](https://www.pyimagesearch.com/) as their code sees judicious use of the `imutils` library. Documentation for `imutils` can be found: [here](https://github.com/jrosebr1/imutils).

#### 1.2.4 Time <a name="1.2.4"></a>

[go to top](#top)

```python
import time
```

`time` is a very useful library overall but especially when dealing with image post-processing work. It can be used to generate the evaluation time per frame as well as used as the file name for output images.

*Tip: If the script requires an FPS counter, you can simply use `imutils` instead of making one on your own using the `time` library. It will help you to save... time.*

## 2 Image I/O <a name="2"></a>

[go to top](#top)

### 2.1 Image Read <a name="2.1"></a>

[go to top](#top)

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

### 2.2 Image Write <a name="2.2"></a>

[go to top](#top)

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

### 2.3 Supported image extensions <a name="2.3"></a>

[go to top](#top)

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

## 3 Video I/O <a name="3"></a>

[go to top](#top)

### 3.1 Webcam / USB Connected Camera <a name="3.1"></a>

[go to top](#top)

To get footage from a webcam or USB connected camera, it is necessary to first define the capture object (the convention is to call it `cap`). Then invoke the `.read()` subroutine on the capture object to yield 1 frame.

```python
cap = cv2.VideoCapture(0) # Set the capture object

ret, frame = cap.read() # Returns a frame

cap.release() # Release the capture object
```

Firstly, the `VideoCapture` object is set to `0`. By default, the first webcam connected to the computer will be `0`. The number can be changed if there are multiple webcams and a specific one has to be connected to. Additionally, if a script connected to a camera fails to exit properly or release the capture object properly, you might find that the capture object number has to be incremented on the subsequent runs. Due to this, ensure that scripts always exit properly and release the capture object before doing so.

`.read()` returns a return, `ret` as well as the frame, `frame`. `ret` and `frame` are merely common conventions.

### 3.2 IP Camera <a name="3.2"></a>

[go to top](#top)

It is also possible to connect to an IP camera as OpenCV supports FFMPEG via HTTP or RTSP. Simply feed the relevant URL into the `VideoCapture` object. [ispyconnect](https://www.ispyconnect.com/) has a good list of camera URLs and their open source utility can be used to find the appropriate URL of the camera as well.

#### 3.2.1 Process <a name="3.2.1"></a>

[go to top](#top)

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

#### 3.2.2 Troubleshooting Notes <a name="3.2.2"></a>

[go to top](#top)

If the URL does not seem to work, do not worry - as long as the URL works on browser or in ispyconnect, the actual URL cannot be far off.

1. For RTSP, generally only the login details and IP are required. So if there are extra strings behind the URL, try removing them to see if it works.

   ```python
   cap = cv2.VideoCapture(http://admin:12345@192.168.0.11/MediaInput) # This might not be working
   
   cap = cv2.VideoCapture(http://admin:12345@192.168.0.11) # But this might work
   ```

2. Check that the port is correct - sometimes the URL will require a port. Remove the port if there is one and try to find a port if there isn't one.
3. Double check the URL - ensure that you are not using HTTPS or that the RTSP link does not start with a HTTP header by mistake.

### 3.3 Full Video Display Script <a name="3.3"></a>

[go to top](#top)

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

### 3.4 Video Writing <a name="3.4"></a>

[go to top](#top)

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

## 4 Image Related Functions <a name="4"></a>

[go to top](#top)

### 4.1 Shape <a name="4.1"></a>

[go to top](#top)

The shape attribute returns the rows, columns and channels of the image (in that order).

```python
img_height = image.shape[0]
img_length = image.shape[1]
img_layers = image.shape[2]
```

`.shape[0]` and `.shape[1]` are some of the most commonly used attributes. It is easy to confuse the two so do refer back when troubleshooting code.

### 4.2 Slicing Images <a name="4.2"></a>

[go to top](#top)

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

## 5 Keystrokes <a name="5"></a>

[go to top](#top)

OpenCV uses the `waitKey` function to draw new frames on to any OpenCV active windows. This means that the `waitKey` function has 2 secondary purposes

- Primary Function: Update the window 
- Secondary Function 1: Allows developer to control the delay of the update loop
- Secondary Function 2: Allows developer to check for specific keystrokes to do specific actions

### 5.1 Close Program <a name="5.1"></a>

[go to top](#top)

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

### 5.2 Adding Additional User Functionality with Keystrokes <a name="5.2"></a>

[go to top](#top)

Most people have taken to appending `if cv2.waitKey(1) & 0xFF == ord('q'):` automatically to the end of their loops. But I personally think that the next approach, while more verbose is superior. It is more extensible and builds a better habit than appending a line without thinking about what it actually means.

In the case where you'd like to add a functionality to `c`apture an image in a video stream, the `waitKey` function can also be used to help detect the `c` keystroke just like it was used to detect the `q` keystroke as a break condition.

```python
key = cv2.waitKey(1) & 0xFF

if key == ord("q"):
    break
elif key == ord('c'):
	cv2.imwrite('output.jpg', image)
```

#### 5.2.1 Working with Special Keys <a name="5.2.1"></a>

[go to top](#top)

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

## 6 Working with Picamera <a name="6"></a>

[go to top](#top)

The RPi Picamera is unique as it connects directly to the RPi using a ribbon cable. Due to this it requires use of the `picamera` library (to be installed separately) and does not use the `VideoCapture` object.

### 6.1 Imports <a name="6.1"></a>

[go to top](#top)

`picamera` generally requires 2 imports in order to work.

```python
from picamera.array import PiRGBArray
from picamera import PiCamera
```

### 6.2 Initialization <a name="6.2"></a>

[go to top](#top)

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

#### 6.2.1 Additional Parameters <a name="6.2.1"></a>

[go to top](#top)

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

##### 6.2.1.1 Setting White Balance <a name="6.2.1.1"></a>

[go to top](#top)

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

### 6.3 Reading Video from Pi Camera <a name="6.3"></a>

[go to top](#top)

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

#### 6.3.1 Video Formats <a name="6.3.1"></a>

[go to top](#top)

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

#### 6.3.2 Flushing the Frame Buffer <a name="6.3.2"></a>

[go to top](#top)

Occasionally in certain use cases, the output framer-ate would significantly stutter, dropping to as low as 2 FPS where one would expect 24 or 30 FPS. On possible solution is to simply flush the frame buffer. In most cases, there will be no noticeable hit to performance if the actual camera frame-rate is high (24 or higher) and the buffer is set low (5 or so frames).

```python
for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    for i in range(5): # Clears the 5 frame buffer 
        frame = img.array
        
    frame = img.array
    
    rawCapture.truncate(0)
```

### 6.4 Scripts <a name="6.4"></a>

[go to top](#top)

The following are the scripts available in the sample code/template code folder:

- `pi_camera_single_frame.py`: Grabs a single pi camera frame and displays it. Can be used to test if the pi camera even works
- `pi_camera_video.py`: Grabs video output from the pi camera. Has various options for the camera as well as the option to flush the frame buffer. Also allows you to capture the active frame by pressing `c`.

## 7 Drawing Functions <a name="7"></a>

[go to top](#top)

Drawing functions are basically the bread and butter of OpenCV. It allows you to highlight important information for client as well as do debugging. Detailed documentation of the drawing functions is available [here](https://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html).

### 7.1 Rectangle <a name="7.1"></a>

[go to top](#top)

Rectangle functions can be used to draw bounding boxes, region of interest, etc.

```python
# Syntax
cv2.rectangle(image, pt1, pt2, color, thickness)

# Example
cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 1)
```

This draws a rectangle from `x1`, `y1` coordinates to `x2`, `y2` coordinates. The tuple after defines the color of the rectangle in bgr format. The last number refers to the thickness.

### 7.2 Text <a name="7.2"></a>

[go to top](#top)

Text can be used to draw all sorts of useful information such as class names, confidence levels or even instructions to the user on how to use the utility.

```python
# Syntax
cv2.putText(image, text, pt, font, fontScale, color, thickness)

# Example
cv2.putText(frame, "hi", (x, y), cv2.FONT_HERSHEY_PLAIN, fontScale=2.0, color=(255, 0, 0), thickness=2)

```

Pretty straightforward syntax-wise. It basically tries to put the text (in this case 'hi') on the frame at the coordinates `x` and `y` with the font `FONT_HERSHEY_PLAIN` with the scale of `2.0`, blue color and the line thickness of 2 pixels.

#### 7.2.1 Font Options <a name="7.2.1"></a>

[go to top](#top)

There are other font options for OpenCV. Usually the font is declared at the top of the program like so:

```python
font = cv2.FONT_HERSHEY_PLAIN

cv2.putText(frame, "hi", (x, y), font, fontScale=2.0, color=(255, 0, 0), thickness=2)
```

Other font options include the following:

- `FONT_HERSHEY_SIMPLEX`
- `FONT_HERSHEY_PLAIN`
- `FONT_HERSHEY_DUPLEX`
- `FONT_HERSHEY_COMPLEX`
- `FONT_HERSHEY_TRIPLEX`
- `FONT_HERSHEY_COMPLEX_SMALL`
- `FONT_HERSHEY_SCRIPT_COMPLEX`
- `FONT_HERSHEY_SCRIPT_SIMPLEX`

Here is an image preview of what the fonts look like (image from [codeyarns](![Fonts available in OpenCV](https://codeyarns.files.wordpress.com/2015/03/20150311))):

![Fonts available in OpenCV](https://codeyarns.files.wordpress.com/2015/03/20150311_opencv_fonts.png?w=1100)

#### 7.2.2 Common Text Locations <a name="7.2.2"></a>

[go to top](#top)

It is common to put text in the edges of the image (such as FPS counters). Due to this, the coordinates of the text can be dynamically obtained using the `.shape` attribute. Examples below:

```python
cv2.putText(frame, fps,  (10, (frame.shape[0] - 20)), cv2.FONT_HERSHEY_PLAIN, fontScale=2.0, color=(0, 0, 255), thickness=2) # fps counter placed relative to the frame's height
```

Take note that text is drawn with the top left of the text at the specified coordinates, thus the need to subtract 20px from the height of the image.

*Reminder: Origin of the image is in the top left corner of the image*

### 7.3 Line <a name="7.3"></a>

[go to top](#top)

Lines are another common entity used in image processing outputs. It can be used to visualize the ROI or even visualize velocities and trends of tracked objects.

```python
# Syntax
cv2.line(image, (pt1), (p2), color, thickness)

# Example
cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
```

Similar to a rectangle except that it draws a straight line between the 2 points being referenced.

### 7.4 Circle <a name="7.4"></a>

[go to top](#top)

Circles are primarily used to draw houghcircles or dots (by drawing a tiny circle). Dots can be used to mark the center of bounding boxes for tracking or other purposes.

```python
# Syntax
cv2.circle(image, pt, radius, color, thickness)

# Example
cv2.circle(frame, (x, y), 5, (255, 0, 0), 2)
```

### 7.5 Other Drawing Functions <a name="7.5"></a>

[go to top](#top)

There are other drawing functions but as they are not as commonly used, they will not be discussed within the scope of this document. Full drawing documentation available [here](https://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html).

## 8 Hough Lines Transform <a name="8"></a>

[go to top](#top)

Hough lines is a line detection method that works as an additional layer after a canny edge filter. This allows straight edges in an image to be detected.

It is a rather expensive algorithm to run and is susceptible to noise, thus the input image has to have a clean signal and not have too many possible lines unless real-time performance is not a requirement.

Full Hough Lines Transform theory and documentation [here](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html).

The following is a Hough Line Transform used to detect a grid and subsequently a lit square in the grid. Notice how the grid can still be properly detected despite it being non-uniform in spacing. This is possible when hough line transform is used instead of template matching.

![1563894456180](assets/1563894456180.png)

### 8.1 Basic Hough Lines Transform Implementation <a name="8.1"></a>

[go to top](#top)

```python
import cv2
import numpy as np

img = cv2.imread('test.jpg')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # convert the image to gray and use a canny edge detector
edges = cv2.Canny(gray,50,150,apertureSize = 3)

lines = cv2.HoughLines(edges,1,np.pi/180,200)
for rho,theta in lines[0]:
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))

    cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imshow("detected lines", img)
```

Notice that the Hough Lines detector actually only gives the rho and theta of the lines. This means that it is possible to generate what type of line or even the equation of the lines from the given output. This information will be of use in the arrow detection implementation as outlined in the Miscellaneous section.

## 9 Hough Circles Transform <a name="9"></a>

[go to top](#top)

Like Hough Lines Transform, Hough Circles Transform simply attempts to find circles in the environment. 

Hough Circles Transform can be quite unreliable and susceptible to noise. Additionally it requires the circles to be circular when viewed head on (as is it's output). It is incapable of detecting warped circles or even circles with obscured parts (or parts out of the frame). This means that it can be a rather unreliable algorithm for certain use cases.

See the circumcenter detection section under miscellaneous for a possible algorithm that might fit your use case.

### 9.1 Simple Demo<a name="9.1"></a>

[go to top](#top)

This demo is taken from the [OpenCV docs](https://docs.opencv.org/master/da/d53/tutorial_py_houghcircles.html).

```python
import numpy as np
import cv2 as cv
img = cv.imread('test.jpg',0)
img = cv.medianBlur(img,5)
cimg = cv.cvtColor(img,cv.COLOR_GRAY2BGR)
circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,20, param1=50,param2=30,minRadius=0,maxRadius=0)
circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    # draw the outer circle
    cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
cv.imshow('detected circles',cimg)
cv.waitKey(0)
cv.destroyAllWindows()
```

The result is as seen below:

![houghcircles2.jpg](https://docs.opencv.org/master/houghcircles2.jpg)

## 10 Morphology Transforms <a name="10"></a>

[go to top](#top)

Morphology transforms are very useful in image processing to bring out certain features or reducing unwanted noise to create a stronger signal.

### 10.1 Preparing an Image for Morphology Transforms <a name="10.1"></a>

[go to top](#top)

#### 10.1.1 Convert to Grayscale<a name="10.1.1"></a>

[go to top](#top)

Morphology transforms can only be be applied to grayscale images. One way to get a grayscale image is to simple convert to grayscale.

This can be done using the `.cvtColor()` function:

```python
gray_image = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
```

#### 10.1.2 Image Thresholding <a name="10.1.2"></a>

[go to top](#top)

The other more common way to get a gray image is to simply do some form of thresholding to clean up the signal in the first place. By doing so, the image becomes cleaner as noise is filtered and a grayscale image is obtained automatically.

Here is a simple example of how to apply a binary threshold:

```python
ret, thresh_img = cv2.threshold(raw_img,10,255,cv2.THRESH_BINARY)
```

`ret` is simply the return (`True` / `False`)  while `thresh_img` (or more commonly simply left as `th`) is the thresholded single-channel image.

#### 10.1.3 Kernel <a name="10.1.3"></a>

[go to top](#top)

Morphology Transforms first require the selection of a kernel. There are various kernels available. In the case of the arrow, a large cross kernel was used and because of that, you can see that the middles of the arrow have been removed.

Selection of the right kernel and size can be tricky but it is necessary process as each use case is different. Start with a small kernel and then increase the size if the results look promising.

Some example kernels:

```python
kernel = np.ones((5,5),np.uint8) # Creates a 5x5 kernel of 1s
kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(19,19)) # 19x19 cross kernel
```

Documentation on Kernels [here](https://docs.opencv.org/3.4/d4/dbd/tutorial_filter_2d.html). 

**Warning: Do note that kernels have to be odd-number in size**

#### 10.1.4 Inverting an Image <a name="10.1.4"></a>

[go to top](#top)

Eroding an image is not the same as inverting an image and then dilating the image. Nor is dilating an image the same as inverting an image and then eroding the image. Due to this, it is prudent to try inverting an image to see if the desired features can be brought out better than the original image.

Inverting an image is relatively simple - just do a bitwise not operation on itself.

```python
inverted_img = cv2.bitwise_not(raw_img, raw_img)
```

### 10.2 Erode and Dilate <a name="10.2"></a>

[go to top](#top)

The OpenCV functions `.erode()` and `.dilate()` are the simplest of the morphology transforms. These 2 morphological operators are especially useful when trying to create more features in otherwise simple shapes with little lighting changes.

For an illustration, we can use this image of an `i`:

![Original image](https://docs.opencv.org/2.4/_images/Morphology_1_Tutorial_Theory_Original_Image.png)

Applying dilation:

![Dilation result - Theory example](https://docs.opencv.org/2.4/_images/Morphology_1_Tutorial_Theory_Dilation.png)

Inverting the image and applying dilation:

![Left image: original image inverted, right image: resulting dilatation](https://docs.opencv.org/2.4/_images/Morphology_1_Tutorial_Theory_Dilatation_2.png)

Erosion of the original image:

![Erosion result - Theory example](https://docs.opencv.org/2.4/_images/Morphology_1_Tutorial_Theory_Erosion.png)

Inverting the image and applying erosion:

![Left image: original image inverted, right image: resulting erosion](https://docs.opencv.org/2.4/_images/Morphology_1_Tutorial_Theory_Erosion_2.png)

If you'd like to know more about the theory, feel free to head over to the [official docs](https://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html).

Eroding or Dilating an image is rather simple, simply call `.erode()` with the frame and the kernel:

```python
eroded_frame = cv2.erode(image, kernel)
dilated_frame = cv2.dilate(image, kernel)
```

### 10.3 Black Hat and Top Hat Transforms <a name="10.3"></a>

[go to top](#top)

Black Hat and Top Hat are more advanced morphological transforms. In a Black hat transform, the image is dilated and then eroded before being compared to the original image. In a top hat transform the image is eroded and then dilated before being compared to the original image. More detailed information can be found [here](https://docs.opencv.org/2.4/doc/tutorials/imgproc/opening_closing_hats/opening_closing_hats.html).

Instead of talking about the theory, here is a practical example a cross-kernel being used to do a blackhat/tophat transform on an arrow to detect key features:![1563810074231](./assets/1563810074231.png)

By comparing the eroded features, we can see if 2 of the points are on the left or right of the centroid to determine the direction of the arrow.

Black Hat and Top hat require a kernel and grayscale image as well, however the method to call them is different as they are part of the `.morphologyEx` function.

```python
tophat = cv2.morphologyEx(raw_img, cv2.MORPH_TOPHAT, kernel) # Top Hat transform
blackhat = cv2.morphologyEx(raw_img, cv2.MORPH_BLACKHAT, kernel) # Black Hat transform
```

### 10.4 Demo Script <a name="10.4"></a>

[go to top](#top)

It makes no sense to keep talking about the theory when a quick demonstration will allow you to understand how these tools can be used to emphasize features you might need. Thus the demo script should be a good start in trying to find out if any of these morphological transforms are useful for you.

Feel free to find the demo script `morph_transform_demo.py`. The options available are as follows:

- `KERNEL`: define the kernel to be used
- `INVERT`: Sets whether the image should be inverted
- `THRESH`: Sets whether to obtain a grayscale image via thresholding or by color conversion to grayscale

## 11 Centroid Detection <a name="11"></a>

[go to top](#top)

Centroid detection is a method to find the "center of mass" of a detected blob. This can be useful when trying to determine the center of a uniform object or used to track the position or velocity of an object that does not have a fixed form (relative to the camera or otherwise).

### 11.1 Contour Detection <a name="11.1"></a>

[go to top](#top)

Centroid detection starts with Contour detection:

```python
img2, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
```

The most important return of interest are the contours found in `contour`

### 11.2 Centroid Detection of the Contours <a name="11.2"></a>

[go to top](#top)

It is then possible to iterate through each contour and find the centroid. It is also possible to filter the contours such that only the largest contour is processed.

```python
for c in contours:
    if cv2.contourArea(c) > minimumArea:
        # calculate moments for each contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(frame, (cX, cY), 5, (255, 100, 255), -1)
        cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.drawContours(frame, [c], -1, (255, 100, 255), 2)
        x,y,w,h = cv2.boundingRect(c)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        rX = int(x+(w/2))
        rY = int(y+(h/2))
        cv2.circle(frame, (rX, rY), 5, (0,255,255), -1)
```

In the example, a rectangle was used to enclosed the contour. However a circle could be used if the objects within the use case are spherical in nature.

## 12 Mouse Events <a name="12"></a>

[go to top](#top)

OpenCV allows for mouse events on an active OpenCV window to do a callback to a specific function. This can be used to allow users to set ROI, select pixels or just add additional functionality.

*As a side note, it's possible to make an entire GUI in OpenCV using rectangles and putText as well as checking mouse click locations - but please don't.*

### 12.1 Bind Callback Function <a name="12.1"></a>

[go to top](#top)

To begin, the callback function has to be bound to a specific OpenCV named window. This can be done by defining the name of the named window before hand and setting the mouse callback.

```python
# Defines some function
def some_function(event):
    pass

cv2.namedWindows('test') # Declares a named wwindow called 'test'
# Attach some_function as a callback for the window 'test'
cv2.setMouseCallback('test', some_function)

# Actually start the window 'test'
cv2.imshow('test', frame)
```

### 12.2 Mouse Event Detection <a name="12.2"></a>

[go to top](#top)

Within the callback function itself, it takes an argument `event` which is the specific mouse event passed onto the function when the callback is invoked. the most basic events needed for most functionality is mouse left button down and mouse left button up.

*Note: It is important to check that the mouse button has been raised or else the functionality enclosed within the mouse button down will take place forever.*

### 12.3 Scripts <a name="12.3"></a>

[go to top](#top)

The example scripts that use mouse events are the `video_crop.py` and `roi_selection_demo.py` scripts. Both of these use mouse events to allow the user to define a specific region of interest.

## 13 SIFT  <a name="13"></a>

[go to top](#top)

Scale-Invariant Feature Transform (SIFT) is a template matching algorithm. The issue with their implementation is that template matching tend to be expensive and only work well in a specific set of circumstances. 

SIFT utilizes feature recognition to match an object in the image to a template that it has as reference. 

### 13.1 SIFT in OpenCV 3 <a name="13.1"></a>

[go to top](#top)

One very common issue with example code on the net for SIFT is that they are often outdated for OpenCV 3. Another possibility is that the programmer might find the module nonexistent.

For a start, SIFT and SURF are both under the opencv-contrib library which is separate from OpenCV itself. Be sure to check that the install/build method that is reference actually clones the opencv-contrib repo and references it during the OpenCV build process.

Additionally, the syntax for SIFT and SURF is a bit different in OpenCV 3 as compared to prior versions.

**OpenCV Version < 3:**

```python
import cv2
detector = cv2.FeatureDetector_create("SIFT")
```

**OpenCV Version 3 and up:**

```python
import cv2
detector = cv2.xfeatures2d.SIFT_create(1000)
```

Hopefully if you already know how to use SIFT and SURF, this has resolved the number 1 issue you've had going into this section.

### 13.2 Using SIFT  <a name="13.2"></a>

[go to top](#top)

```python
frame = cv2.imread('test.png', 0) # read the test image in gray

ref_img = cv2.imread('ref.png',0) # read the reference image in gray
#  Initiate SIFT detector
sift1 = cv2.xfeatures2d.SIFT_create(1000)
sift2 = cv2.xfeatures2d.SIFT_create(1000)

# Run feature detection on both images
kp1, des1 = sift1.detectAndCompute(frame,None)
kp2, des2 = sift2.detectAndCompute(ref_img,None)
# FLANN parameters
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 2)
search_params = dict(checks=100)   # or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params,search_params)
matches = flann.knnMatch(des1,des2,k=2)
# Need to draw only good matches, so create a mask
matchesMask = [[0,0] for i in xrange(len(matches))]
# ratio test as per Lowe's paper
matched_f = 0
for i,(m,n) in enumerate(matches):
    if m.distance < 0.62*n.distance:
        matched_f += 1
        matchesMask[i]=[1,0]
        draw_params = dict(matchColor = (0,255,0),
                           singlePointColor = (255,0,0),
                           matchesMask = matchesMask,
                           flags = 0)
final_img = cv2.drawMatchesKnn(img_rec_red2,kp1,img2,kp2,matches,None,**draw_params)

cv2.imshow('image',final_img)
```

So what was all that? Basically, 2 sift objects were created to find features in both the reference and the actual image. Then the points were compared to see if enough features were similar. The output is then shown.

Of course the theory is more complex but the idea here is to show how it works and provide example code for you to get going.

### 13.3 When not to use SIFT <a name="13.3"></a>

[go to top](#top)

Template matching is a pretty powerful tool, but there is a reason why I have left it to one of the last few sections. This is because template matching should not be used as a crutch. It is slow, unreliable and generally gets affected by too much noisy in complex environments. Where possible, you should always aim to use more efficient or generic algorithms instead of template matching - it should be a last resort.

## 14 Camera Calibration <a name="14"></a>

[go to top](#top)

At times, the use case will call for a fish-eye camera or even simply a lens that has an FOV beyond 80 degrees - causing a slightly distortion effect. While this should not affect the more robust or general algorithms, it can cause issues with tracking or detection models that are more sensitive to object size, shape and the like. Especially when it comes to extreme fish-eye lenses where the same object can appear to warp and stretch differently at different parts of the image.

To circumvent this, it is possible to actually map each pixel in the image to another location such that image becomes undistorted. This would obviously result in the edges of the image to have all sorts of artifacts but for the most part the corrected image will be able to be used for image processing purposes.

To obtain the this "map" or more correctly known as a camera calibration matrix, the camera has to be calibrated using a grid of black and white (checkerboard pattern). This is because the spacing and "groundtruth" of a checkerboard pattern can be easily created in OpenCV (to the point that there is a function to generate this groundtruth). By comparing the groundtruth and the actual checkerboard pattern detected in the image, the camera calibration matrix can be generated.

### 14.1 Checkerboard Calibration <a name="14.1"></a>

[go to top](#top)

The checkerboard calibration involves taking many images of the checkerboard at various locations within the frame. The images are then globbed and converted to grayscale before `.findChessboardCorners` is invoked to find the checkerboard corners.

```python
ret, corners = cv.findChessboardCorners(gray, (7,6), None)
```

In this case `(7,9)` means that the checkerboard is 7x9 in size.

### 14.2 Getting the camera `mtx` <a name="14.2"></a>

[go to top](#top)

The camera `mtx` is then obtained by using the corners from all detected checkerboards:

```python
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
```

`mtx` in this case refers to the camera calibration matrix.

### 14.3 Getting the Optimal Camera Matrix for Undistortion <a name="14.3"></a>

[go to top](#top)

With the `mtx` and `dist` matrices, the optimal camera matrix can be generated and invoked by `.undistort` to yield the undistorted image.

```python
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width,height), 1, (width,height))
undistort = cv2.undistort(frame, mtx, dist, None, newcameramtx)
```

### 14.4 Steps for Demo <a name="14.4"></a>

[go to top](#top)

2 demo scripts are available for trying out the OpenCV undistort functionality.

1. Print out the checkerboard pattern from [mrpt.org](https://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf) or whichever checkerboard pattern desired
2. Use an image capture script to capture images of the checkerboard pattern at different locations in the frame but perpendicular to the camera lens (capture at least 30 images)
3. Save these images within a folder and then run `checkerboard_calib.py`. The `ret, mtx, dist, rvecs, tvecs` will be printed in the console. These value will be important
4. Enter the values into the `camera_undistort_demo.py` and run the script

## 15 Aruco Marker Detection <a name="15"></a>

[go to top](#top)

Aruco markers are little markers that look like QR codes. They are very robust and thus where possible, they can be used as the preferred method of object detection or even localization (it is possible to estimate pose from Aruco markers).

First the OpenCV aruco dictionary is initialized based on aruco size.

You can generate aruco markers of appropriate sizes at http://chev.me/arucogen/.

```python
import cv2.aruco as aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250) # initializing a dictionary for 4x4 aruco markers
```

Then the image is converted to gray and the detection subroutine is called.

```python
parameters = aruco.DetectorParameters_create()

corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
```

The `aruco_detector.py` script is available and implemented as a class to allow easy import and integration into other code. Run it as a standalone script with a local file named `test.jpg` to test the code.

## 16 Miscellaneous Tricks and Scripts <a name="misc"></a>

[go to top](#top)

### 16.1 Limit Display FPS <a name="misc.1"></a>

[go to top](#top)

This allows the display loop to be set at a specific desired FPS.

```python
desired_FPS = 30

img_ms = int(1000 / desired_FPS)
if cv2.waitKey(img_ms) & 0xFF == ord('q'):
    break
```

### 16.2 Video Cropping Utility <a name="misc.2"></a>

[go to top](#top)

`video_cropper.py` is a script that allows the user to set an ROI and then output a cropped version of the input video.

Simply define the input and output paths for the videos and then launch the utility. 

Use the mouse to select the ROI:

- Press `r` to reset the ROI
- Press `c` to confirm the ROI an begin the crop process

### 16.3 ROI Selection Utility <a name="misc.3"></a>

[go to top](#top)

Within the example code is the `roi_selection_demo.py` script. This script allows the user to select 4 points to form an ROI. In the demo, the points are only printed in the console as output after being 'saved'. But that segment of the code can be modified as necessary.

The main considerations are to allow the user to select ROI beyond the scope of the image so as to allow them to select the shape they really want. Additionally, if the image is too large, it will be scaled down accordingly so that the user can actually see the whole image on a normal monitor without investing in 4K.

The key takeaways are that:

- OpenCV accepts negative coordinates and will compute them properly relative to the origin (top left of the image)
- Images and coordinates can be scaled up or down with almost no loss in accuracy.

An advanced version of this code was used as part of an ROI selection utility during my internship stint. 

### 16.4 Use Odd Shaped ROI <a name="misc.4"></a>

[go to top](#top)

This works in conjunction with the ROI selection utility in the previous section as the user is likely to have selected a weird ROI.

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

### 16.5 Working with Model Bounding Boxes <a name="misc.5"></a>

[go to top](#top)

When working if Neural Network models and the like, the output is usually a list of bounding boxes. Generally, the bounding boxes can be of 2 types of formats:

- `[x, y, w, h]`
- `[x1, y1, x2, y2]`

By my own conventions, I refer to them as convention `0` and convention `1` respectively.

To allow swapping of models within the code, it would be good to take into account the 2 possible conventions.

Functions to draw bounding boxes for both types of conventions can be found in `draw_bounding_boxes.py`.

### 16.6 Circle center detection using Circumcenter <a name="misc.6"></a>

[go to top](#top)

This is an alternative algorithm to Hough Circles Transform if you use case is as follows:

- There is only 1 circle to be detected
- The signal can be isolated such that only parts of the circle are detected and nothing else
- Parts of the circle may be obscured but the center of the circle still has to be located

This algorithm takes 3 points from what it assumes to be parts of the circle (this is why a clean signal of the circle and nothing but the circle is required). It sorts all possible candidate points and splits the 'circle' into 3 sections - top, middle, bottom and chooses one random point in each section.

Then using the [circumcenter formula](https://www.vedantu.com/formula/circumcenter-formula), it attempts to compute the center of the circle.

The demo implementation can be found in the sample code as `circumcenter_demo.py`. It requires `sympy` to run. It also assumes that you have a grayscale image with a strong signal to detect with.

### 16.7 Arrow Detection with Hough Lines Transform <a name="misc.7"></a>

[go to top](#top)

To determine the direction of an arrow, template matching or other methods could be used. However one approach could see the use of Hough Lines Transform.

```python
for rho,theta in line:
        if theta > 0.698 and theta < 1.047:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho

            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))


            x_cood = int(-(rY - (rho/b)) / (a/b))
            print(x_cood,rY)
        
            cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.circle(frame, (x_cood, rY), 5, (0,255,255), -1)
        elif theta > 2.269 and theta < 2.443:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            x_cood = int(-(rY - (rho/b)) / (a/b))
            print(x_cood,rY)
            cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
            cv2.circle(frame, (x_cood, rY), 5, (0,255,255), -1)
```

By checking `rho` and `theta`, the lines forming the arrow head can be categorized. `rY` is the y coordinate of the centroid. the value of `rY` is then substituted into the equation of the line to get the `x_cood` of the point had it followed the line.

By comparing the x coordinate of `rY` on the line and the tru x coordinate of the centroid of the arrow. It is possible to determine if the arrow was pointed to the left or right.

Unfortunately the arrow test script is an unedited full script of a drone flight test script. Feel free to reference it however - I will edit it in the future to isolate a demo snippet if I have the time.

### 16.8 Red Color Thresholding Using RGB <a name="misc.8"></a>

[go to top](#top)

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