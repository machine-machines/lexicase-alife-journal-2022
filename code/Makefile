OS := $(shell uname)
VERSION = $(shell cat VERSION)

BUILD = build

RELEASE = ~/libXeres

FLAGS=-O3 -Wall -std=c++14
DEFINES=
TARGET=libXeres.a
OBJECTS=EvolutionScheme.o WallTask.o TurnTask.o FloorIsLavaTask.o PreciousCargoTask.o Task.o TurnTaskWorld.o WallTaskWorld.o FloorIsLavaTaskWorld.o PreciousCargoTaskWorld.o Species.o TaskWorld.o Agent.o Util.o

ifeq ($(OS), Darwin)
ODE=/Users/user/ode-0.16.2
else
ODE=/home/user/ode-0.16.2
LINK=-lrt
endif

ODEh=-I$(ODE)/include
ODEl=$(ODE)/ode/src/.libs/libode.a -lpthread $(LINK)

release: libXeres test experiment
	mkdir -p $(RELEASE)/include
	mkdir -p $(RELEASE)/lib
	mkdir -p $(RELEASE)/bin
	cp *.h $(RELEASE)/include
	cp *.hpp $(RELEASE)/include
	cp $(BUILD)/$(TARGET) $(RELEASE)/lib
	cp $(BUILD)/test-evaluate $(RELEASE)/bin
	cp $(BUILD)/test-evolve $(RELEASE)/bin
	cp $(BUILD)/experiment $(RELEASE)/bin

cleanrelease: clean build release

libXeres: $(OBJECTS)
	cd $(BUILD) ; \
	ar rvs $(TARGET) $(OBJECTS)

experiment: experiment.o
	cd $(BUILD) ; \
	g++ -o experiment experiment.o -lXeres -L. $(ODEl)

experiment.o: experiment.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/experiment.o experiment.cpp -c $(ODEh)

test: test-evolve.o test-evaluate.o libXeres
	cd $(BUILD) ; \
	g++ -o test-evaluate test-evaluate.o -lXeres -L. $(ODEl) ; \
	g++ -o test-evolve test-evolve.o -lXeres -L. $(ODEl)

test-evolve.o: test-evolve.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/test-evolve.o test-evolve.cpp -c $(ODEh)

test-evaluate.o: test-evaluate.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/test-evaluate.o test-evaluate.cpp -c $(ODEh)

TurnTaskWorld.o: TurnTaskWorld.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/TurnTaskWorld.o TurnTaskWorld.cpp -c $(ODEh)

WallTaskWorld.o: WallTaskWorld.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/WallTaskWorld.o WallTaskWorld.cpp -c $(ODEh)

FloorIsLavaTaskWorld.o: FloorIsLavaTaskWorld.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/FloorIsLavaTaskWorld.o FloorIsLavaTaskWorld.cpp -c $(ODEh)

PreciousCargoTaskWorld.o: PreciousCargoTaskWorld.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/PreciousCargoTaskWorld.o PreciousCargoTaskWorld.cpp -c $(ODEh)

WallTask.o: WallTask.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/WallTask.o WallTask.cpp -c $(ODEh)

TurnTask.o: TurnTask.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/TurnTask.o TurnTask.cpp -c $(ODEh)

FloorIsLavaTask.o: FloorIsLavaTask.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/FloorIsLavaTask.o FloorIsLavaTask.cpp -c $(ODEh)

PreciousCargoTask.o: PreciousCargoTask.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/PreciousCargoTask.o PreciousCargoTask.cpp -c $(ODEh)

EvolutionScheme.o: EvolutionScheme.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/EvolutionScheme.o EvolutionScheme.cpp -c $(ODEh)

Task.o: Task.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/Task.o Task.cpp -c $(ODEh)

TaskWorld.o: TaskWorld.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/TaskWorld.o TaskWorld.cpp -c $(ODEh)

Species.o: Species.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/Species.o Species.cpp -c $(ODEh)

Util.o: Util.cpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/Util.o Util.cpp -c $(ODEh)

Agent.o: Agent.cpp Util.hpp
	g++ $(FLAGS) $(DEFINES) -o $(BUILD)/Agent.o Agent.cpp -c $(ODEh)

clean:
	rm -f test-evaluate test-evolve
	rm -f experiment
	rm -rf $(BUILD)
	rm -f $(TARGET) *.o
	rm -fr $(RELEASE)
	rm -f libXeres-$(VERSION)
	rm -f libXeres-$(VERSION).tar.gz

build:
	mkdir -p $(BUILD)