TARGET = NetsimTraciClientMain
LIBS =
CC = g++
CFLAGS = -g
HEADERS = NetsimTraciClient.h socket.h StdDefs.h storage.h SUMOTime.h TplConvert.h TraCIAPI.h \
TraCIConstants.h UtilExceptions.h ToString.h MSVehicleStateTable.h
OBJECTS = NetsimTraciClientMain.o NetsimTraciClient.o socket.o storage.o SUMOTime.o TraCIAPI.o \
MSVehicleStateTable.o

.PHONY: default all clean

default: $(TARGET)
all: default

%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -Wall $(LIBS) -o $@

clean:
	-rm -f $(OBJECTS)
	-rm -f $(TARGET)