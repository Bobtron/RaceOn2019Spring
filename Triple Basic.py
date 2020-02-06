#!/usr/bin/env python
# coding: utf-8

# In[1]:


from picamera.array import PiYUVArray
from picamera import PiCamera

from scipy.signal import find_peaks, butter, filtfilt
from pwm import PWM
import threading
import ipywidgets as ipw
import time
# import matplotlib.pyplot as plt
import skimage as ski

WIDTH_ = 224
HEIGHT_ = 160
MEDIAN_ = WIDTH_ // 2

# Camera resolution
res = (WIDTH_, HEIGHT_)

# Run a track detection algorithm on a single horizontal line.
# Uses YUV420 image format as the Y component corresponds to image intensity (gray image)
# and thus there is no need to convert from RGB to BW

camera = PiCamera()
        
# Check the link below for the combinations between mode and resolution
# https://picamera.readthedocs.io/en/release-1.13/fov.html#sensor-modes
camera.sensor_mode = 7
camera.resolution = res
camera.framerate = 120

# To filter the noise in the image we use a 3rd order Butterworth filter

# Wn = 0.02, the cut-off frequency, acceptable values are from 0 to 1
b, a = butter(3, 0.02)

pwm0 = PWM(0)
pwm1 = PWM(1)

pwm0.period = 20000000
pwm1.period = 20000000

pwm0.enable = True
pwm1.enable = True


# In[2]:


pwm0.duty_cycle = 1000000
pwm1.duty_cycle = 1600000
time.sleep(1)
pwm1.duty_cycle = 1100000
time.sleep(1)
pwm1.duty_cycle = 2100000
time.sleep(1)
pwm1.duty_cycle = 1600000


# In[3]:


# max speed that this algo works
# ucla = 1120000
# sonic = 1250000

UCLA_ = 1138500
USC_ = 1275000
SONIC_ = 1275000

UPPER_LINE_ = int(HEIGHT_ * 0.95)
CENTER_LINE_ = HEIGHT_ // 2
LOWER_LINE_ = int()
INTERVAL_ = 0.0
CENTER_THRESHOLD_ = WIDTH_ // 10
NO_PEAKS_ = -520

STRAIGHT_ = 1600000
RIGHT_ = 1100000
LEFT_ = 2100000
NUM_PREV_VAL_ = 8

killSwitch = ipw.ToggleButton(value=False, description='Shut it down')
# minMaxSlider = ipw.IntRangeSlider(value=[UCLA_/1000,SONIC_/1000], min = 1000, max = 1500,step = 1, description = 'Speed Ranges')

# display(minMaxSlider)
display(killSwitch)

def main_loop():
    raw_capture = PiYUVArray(camera, size=res)
    stream = camera.capture_continuous(raw_capture, format="yuv", use_video_port=True)

    # for pwm1
    # duty_cycle

    #      (array([482, 542], dtype=int32), {'peak_heights': array([186.831036  , 185.01286929])})

    num_frames = 0

    # if line_pos is 1 then the line is to the right of the car
    # if line_pos is -1 then the line is to the left of the car
    # if line_pos is 0 either uhoh we're doomed
    # if lost is true then the line is lost
    line_pos = STRAIGHT_
    lost = False
    prev_steer_dc = pwm1.duty_cycle
    prev_steer_diff_arr = []
    prev_peak_avg_arr = []

    for i in range(NUM_PREV_VAL_):
        prev_steer_diff_arr.append(-1)
        prev_peak_avg_arr.append(-1)
    
    for f in stream:
        t = time.time()

        # This was intended change speeds as necesary
        # But not anymore since its too volatile, can be manually updated
        # speeds = update_speeds()
        # UCLA_ = speeds[0]
        # SONIC_ = speeds[1]

        # If killSwitch is toggled, breaks out of forever loop and kills the motor
        if killSwitch.value:
            break

        num_frames += 1
        # Get the intensity component of the image (a trick to get black and white images)
        I = f.array[:, :, 0]
        # Reset the buffer for the next image
        raw_capture.truncate(0)
        # Select a horizontal line in the middle of the image
        U = I[UPPER_LINE_, :]
        C = I[CENTER_LINE_, :]
        L = I[LOWER_LINE_, :]
        # Smooth the transitions so we can detect the peaks 
        Lf = filtfilt(b, a, C)
        # Find peaks which are higher than 0.5
        p = find_peaks(Lf, height=128)

        total_peak_amt = 0
        avg_peak_loc = NO_PEAKS_
        num_peaks = 0

        for peak in p[0]:
            num_peaks += 1
            total_peak_amt += peak

        print("Num Peaks: " + num_peaks)

        if num_peaks != 0:
            avg_peak_loc = total_peak_amt / num_peaks
            if num_peaks == 3:
                print("THIS IS THE BEGINNING OF THE END")

        if avg_peak_loc != NO_PEAKS_:
            if avg_peak_loc < MEDIAN_:
                if lost and line_pos == RIGHT_:
                    # If this happens, that means that the center line is to the right
                    # But we encountered the bordering track to the left
                    # Break to stop going over and fucking shit up
                    # TODO: Make clause that handles this, so in case of race it doesn't stop without violating shit
                    break
                line_pos = LEFT_
            else:
                if lost and line_pos == LEFT_:
                    # If this happens, that means that the center line is to the left
                    # But we encountered the bordering track to the right
                    # Break to stop going over and fucking shit up
                    # TODO: Make clause that handles this, so in case of race it doesn't stop without violating shit
                    break
                line_pos = RIGHT_

            # Sets the steering position based on the peak of the Center line
            pwm1.duty_cycle = int(STRAIGHT_ - ((avg_peak_loc - MEDIAN_)/MEDIAN_)*500000)
            lost = False
        else:
            if line_pos == LEFT_:
                pwm1.duty_cycle = LEFT_
            elif line_pos == RIGHT_:
                pwm1.duty_cycle = RIGHT_
            else:
                pwm1.duty_cycle = STRAIGHT_

            lost = True
            # pwm0.duty_cycle = SONIC_

        if abs(prev_steer_dc - STRAIGHT_) < abs(pwm1.duty_cycle - STRAIGHT_) and prev_steer_diff_arr[0] > 0 and prev_steer_diff_arr[1] > 0 and abs(pwm1.duty_cycle - STRAIGHT_) > 400000:
            pwm0.duty_cycle = SONIC_
        elif abs(avg_peak_loc - MEDIAN_) <= CENTER_THRESHOLD_:
            i = 0
            while i < NUM_PREV_VAL_:
                if abs(prev_peak_avg_arr[i] - MEDIAN_) > CENTER_THRESHOLD_:
                    break
                i += 1
            # print("STRAIGHT")
            if i == NUM_PREV_VAL_:
                pwm0.duty_cycle = USC_
        else:
            pwm0.duty_cycle = UCLA_
            if abs(pwm1.duty_cycle - STRAIGHT_) > 300000:
                pwm0.duty_cycle = UCLA_ + 4000

        i = NUM_PREV_VAL_ - 1
        while i > 0:
            prev_steer_diff_arr[i] = prev_steer_diff_arr[i - 1]
            prev_peak_avg_arr[i] = prev_peak_avg_arr[i - 1]
            i -= 1
        
        prev_steer_diff_arr[0] = abs(pwm1.duty_cycle - STRAIGHT_) - abs(prev_steer_dc - STRAIGHT_)
        prev_peak_avg_arr[0] = avg_peak_loc

        prev_steer_dc = pwm1.duty_cycle

        time_elapsed = time.time() - t
        
        print(prev_peak_avg_arr)
        print(prev_steer_diff_arr)
        print(sum(prev_steer_diff_arr))
        print(sum(prev_steer_diff_arr) / len(prev_steer_diff_arr))
        print(pwm1.duty_cycle)
        print("Elapsed {:0.4f} seconds, estimated FPS {:0.2f}".format(time_elapsed, 1 / time_elapsed))
        time.sleep(INTERVAL_)
        
    pwm0.duty_cycle = 1000000

    # Release resources
    stream.close()
    raw_capture.close()
    camera.close()


# In[5]:


'''
def update_speeds():
    UCLA_ = minMaxSlider.value[0] * 1000
    SONIC_ = minMaxSlider.value[1] * 1000
    return [UCLA_, SONIC_]
minMaxSlider.observe(update_speeds)
'''


# In[ ]:


t = threading.Thread(target=main_loop)
t.start()


# In[7]:





# In[ ]:


'''plt.imshow(I)'''


# In[2]:


'''plt.plot(L, label="raw")
plt.plot(Lf, label="filtered")
plt.ylim([0, 300])
plt.legend()'''


# In[1]:


camera.close()


# In[7]:


pwm0.duty_cycle = 1000000


# In[1]:


from pwm import PWM

pwm0 = PWM(0)

pwm0.duty_cycle = 1000000

pwm0.period = 20000000

pwm0.enable = True


# In[ ]:





# In[ ]:





# In[ ]:




