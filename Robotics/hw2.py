####################################
# Homework 2 CSE 494
# Intro Robotics - Dr. Heni Ben Amor
####################################
# NAME: Jeffrey Ciferno
# ASU ID:1217664240            
####################################

# TODO: python3 -m pip install scikit-image matplotlib
import matplotlib.pyplot as plt
from skimage.exposure import rescale_intensity
import matplotlib.gridspec as gridspec
import numpy as np
import cv2

###################################
# BEGIN: PART 1                   #
###################################

def apply_convolution(image, kernel):
    image_height = image.shape[0]
    image_width = image.shape[1]

    kernel_height = kernel.shape[0]
    kernel_width = kernel.shape[1]
    # Create padding for the convolution
    pad = int((kernel_width - 1)/ 2)
    image = cv2.copyMakeBorder(image, pad, pad, pad, pad, cv2.BORDER_REPLICATE)

    # Create an empty image of all zeros to be filled in below
    out_image = np.zeros((image_height, image_width), dtype="float32")

    for y in np.arange(pad, image_height + pad):
        for x in np.arange(pad, image_width + pad):
            roi = image[y - pad:y + pad +1, x - pad:x + pad + 1] #setting region of interest, https://www.pyimagesearch.com/2016/07/25/convolutions-with-opencv-and-python/
            for ky in range(kernel_height):
                for kx in range(kernel_width):
                    k = (roi * kernel).sum()
                    out_image[y - pad, x - pad] = k
                    continue
            # Store result in the out_image

    out_image = rescale_intensity(out_image, in_range=(0, 255))
    out_image = (out_image * 255).astype("uint8")
    return out_image



def executeProblem1(kernel_type):
    # This is where we will store the kernels
    kernels = dict()

    # Create Kernel 1
    sobel_x = np.array(([-1, 0, 1],
                        [-2, 0, 2],
                        [-1, 0, 1]))
    kernels['sobel_x'] = sobel_x

    # Create Kernel 2
    blur_amount = 7 # Number should be odd for convolution
    large_blur = np.ones((blur_amount, blur_amount)) * (1.0 / (10*blur_amount**2))
    kernels['large_blur'] = large_blur

    # TODO: CREATE YOUR SHARPEN FILTER HERE
    sharpen = np.array(([0, -1, 0],
                        [-1, 5, -1],
                        [0, -1, 0]))
    kernels['sharpen'] = sharpen

    # TODO: CREATE YOUR SOBEL_Y FILTER HERE
    sobel_y = np.array(([1, 2, 1],
                        [0, 0, 0],
                        [-1, -2, -1]))
    kernels['sobel_y'] = sobel_y

    # TODO: Make sure your image is available at the following location/path/name:
    # (download rover.png to the current directory from canvas)
    # image source: https://www.pinterest.com/pin/299278337737606357/
    image_location = './rover.png'
    original_image = cv2.imread(image_location)
    # We need to convert a 3-Channel RGB image to 1 Channel
    image_grayscale = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)

    # Select kernel and load the image
    kernel = kernels[kernel_type]

    # Apply the convolution using the selected kernel
    our_output = apply_convolution(image_grayscale, kernel)
    opencv_output = cv2.filter2D(image_grayscale, -1, kernel)

    return image_grayscale, opencv_output, our_output



###################################
# BEGIN: PART 2                   #
###################################

def executeProblem2(measurements):
    # Smoothing Parameter
    K = 0.2
    # Initialize the state with the first measurement value
    state = measurements[0]
    # Generate a placeholder array for filtered data
    filtered = np.empty(len(measurements))
    filtered[0] = state
    for index, measurement in enumerate(measurements):
            #print(index)
            state = K*measurement + (1-K)*state
            filtered[index] = state
    return filtered


###################################
# BEGIN: PART 3                   #
###################################

def executeProblem3(measurements, sensor_variance):
    # Gain Parameter
    K = 0.0
    # State Variance
    P = 1
    # Sensor Variance
    R = sensor_variance
    # Initialize State Value
    xk = measurements[0]
    # Generate a placeholder array for filtered data
    filtered = np.empty(len(measurements))
    pk = np.empty(len(measurements))
    # Initialize Filtered Value
    filtered[0] = xk
    pk[0] = P
    for index, measurement in enumerate(measurements):
        #prediction
        if index != 0: #catch data at index 0 
            xkk = filtered[index-1]
            pkk = pk[index-1] 
        else:
            xkk = filtered[index]
            pkk = pk[index]
        #print("pkk")
        #print(pkk)

        #measurement
        kk = (pkk/(pkk + R))

        #print out checks for debug
        #print("kt")
        #print(kk)
        #print("xkk")
        #print(xkk)

        xk = xkk + kk*(measurement - xkk)
        filtered[index] = xk
        pk[index] = (1-kk)*pkk

    return filtered


###################################
# Data Generation for Problem 2,3 #
###################################

def getMeasurementData(mu, sigma, num_samples):
    # Generate N random samples from Normal(mu, sigma)
    measurements = np.random.normal(mu, sigma, num_samples)
    return measurements


###################################
# Plot Generation for Main Script #
###################################

def plotData(measurement_data, mean, sharpen_output, sobel_y_output,
             moving_average_data, kalman_filter_data):
    # Configure Grid
    fig1 = plt.figure(1, constrained_layout=True)
    main_gs = gridspec.GridSpec(4, 4, figure=fig1)
    sharpen_gs = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=main_gs[:,2])
    sobel_y_gs = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=main_gs[:,3])
    filter_gs = main_gs[0:4,0:2].subgridspec(2,1)

    # Plot the Moving Average Results and Measurements
    ax1 = fig1.add_subplot(filter_gs[0,:])
    ax1.plot(measurement_data, 'r', lw=1)
    ax1.plot(moving_average_data, 'g', lw=2)
    ax1.axhline(measure_mean, color='blue', lw=1)
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Feet')
    ax1.set_title('Top Plot: Moving Average')

    # Plot the Kalman Filter Results and Measurements
    ax2 = fig1.add_subplot(filter_gs[1,:])
    ax2.plot(measurement_data, 'r-', lw=1)
    ax2.plot(kalman_filter_data, 'g-', lw=2)
    ax2.axhline(measure_mean, color='blue', lw=1)
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Feet')
    ax2.set_title('Bottom Plot: Kalman Filter')

    # Plot Image Data
    img_labels = ['Original Image', 'OpenCV\'s output', 'Our Output']
    for i in range(3):
        ax_sobel = fig1.add_subplot(sobel_y_gs[i])
        ax_sobel.imshow(sobel_y_output[i], cmap='gray')
        ax_sobel.set_xlabel(img_labels[i] + ' (Sobel Y)')
        ax_sobel.set_xticks([])
        ax_sobel.set_yticks([])
        ax_sharpen = fig1.add_subplot(sharpen_gs[i])
        ax_sharpen.imshow(sharpen_output[i], cmap='gray')
        ax_sharpen.set_xlabel(img_labels[i] + ' (Sharpen)')
        ax_sharpen.set_xticks([])
        ax_sharpen.set_yticks([])

    plt.show()


################################################################
# Debug Problem 1 Output (For testing only)                    #
# NOTE: WHEN SUBMITTING, DO NOT HAVE ANY CALLS TO THIS METHOD! #
################################################################

def showProblem1(problem1_output):
    cv2.imshow("Original Image", problem1_output[0])
    cv2.imshow('Output from Open CV\'s convolution', problem1_output[1])
    cv2.imshow('Output from Our convolution', problem1_output[2])
    cv2.waitKey(0)
    cv2.destroyAllWindows()


###################################
# MAIN ENTRY POINT                #
###################################

if __name__ == '__main__':
    # Execute Problem 1: sharpen
    sharpen_output = executeProblem1(kernel_type='sharpen')
    #showProblem1(sharpen_output)
    # Execute Problem 1: sobel_y
    sobel_y_output = executeProblem1(kernel_type='sobel_y')
    #showProblem1(sobel_y_output)
    # HW Instructions Snippet
    #sobel_x_output = executeProblem1(kernel_type='sobel_x')
    #showProblem1(sobel_x_output) # Method to debug Problem 1

    # Measurement Data Parameters for generating random data
    measure_mean = 6
    measure_sd = 2
    N = 300
    measurement_data = getMeasurementData(mu=measure_mean,
                                          sigma=measure_sd,
                                          num_samples=N)

    # Execute Problem 2
    moving_average_data = executeProblem2(measurements=measurement_data)

    # Execute Problem 3
    kalman_filter_data = executeProblem3(measurements=measurement_data,
                                         sensor_variance=measure_sd**2)

    # Final Submission Plot
    plotData(measurement_data, measure_mean, sharpen_output,
             sobel_y_output, moving_average_data, kalman_filter_data)

