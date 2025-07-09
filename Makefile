CXX=avr-g++
CXX_FLAGS=-Os
MCU_FLAGS=\
					-mmcu=atmega2560 \
					-D F_CPU=16000000UL \

ARDUINO_INCLUDE=\
								-D ARDUINO_ARCH_AVR \
								-D ARDUINO_AVR_MEGA2560 \
								-I deps/ArduinoCore-avr/cores/arduino/ \
								-I deps/ArduinoCore-avr/variants/mega/ \

# Find all implementation files excluding non AVR archs (eg: samd)
EXECUTABLES= ${shell find src -type f -name *.cpp \
						 -not -path */samd_*}

.PHONY: clean

all: ${patsubst src/%.cpp,build/%.o,${EXECUTABLES}} | build


build/%.o: src/%.cpp | build
	mkdir -p ${dir $@}
	${CXX} ${CXX_FLAGS} ${MCU_FLAGS} ${ARDUINO_INCLUDE} -c\
	 	$< -o $@

build:
	mkdir build

clean:
	rm -rf build/*
