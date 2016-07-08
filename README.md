# Traffic assitant system

# Build and install of project and dependency
## Opencv download,build and install
To install all dependency for opencv 3.1.0 this packages need to be install

``` sudo apt-get install build-essential git cmake pkg-config libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libgtk2.0-dev libatlas-base-dev gfortran python2.7-dev python3-dev python3.4-dev python-pip python3-pip  && sudo pip install --upgrade numpy && sudo pip3 install --upgrade numpy```

Then opencv 3.1.0 download should be do as (This method is tested other build flags should be works too, We implemented all of our code to compatible with 3.1.0 and 3.0.0)

``` wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.1.0.zip &&  wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.1.0.zip && unzip opencv.zip && opencv_contrib.zip  && cd opencv && mkdir build && cd build ```

We use this cmake command (Changing this flags could create some errors or undefined behaviour in our code)

``` cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_C_EXAMPLES=OFF 	-D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=/home/orangepi/opencv_contrib/modules -D BUILD_EXAMPLES=ON -D PYTHON3_LIBRARY=/usr/lib/arm-linux-gnueabihf/libpython3.4m.so -D PYTHON3_PACKAGES_PATH=/usr/lib/python3.4/site-packages/ -D PYTHON3_INCLUDE_DIR=/usr/include/python3.4m -D PYTHON3_EXECUTABLE=/usr/bin/python3 -D BUILD_opencv_python3=ON .. && make -j4 && sudo make install && sudo ldconfig ```

## WiringPi install
To install Wiringpi:

``` git clone https://github.com/zhaolei/WiringOP.git -b h3 && cd WiringOP && chmod +x ./build && sudo ./build && echo /etc/ld.so.conf > /usr/local/lib && sudo ldconfig && cd wiringPi && make static && sudo make install-static ```

## Building Project
To make our code you can use Makefile as
``` make ```

## Running Program
``` sudo ./vehicleSystem ``` (sudo permission needed for accessing and modifiying gpio pins state and values)