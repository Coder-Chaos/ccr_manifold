CC = gcc
CXX = g++

LIBS += -L . libdvp.so libhzd.so -lpthread -Wl,-rpath=.

Demo:Demo.cpp
	$(CXX) -o $@ $< $(LIBS) 

clean:
	rm Demo 
