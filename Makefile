all:
	g++ -o vehicleSystem -Wall `pkg-config --cflags opencv` -O3 main.cpp hardware.c `pkg-config --libs opencv` -lwiringPi

clean:
	rm *.out *.o vehicleSystem
