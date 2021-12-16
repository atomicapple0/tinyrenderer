SYSCONF_LINK = g++
CPPFLAGS     =
LDFLAGS      =
LIBS         = -lm

DESTDIR = ./
TARGET  = main

OBJECTS := $(patsubst %.cpp,%.o,$(wildcard *.cpp))
ifeq ($(OS),Windows_NT)
	RM = del
else
	RM = rm -f
endif

all: $(DESTDIR)$(TARGET)
	main.exe
	C:\Program Files (x86)\XnView\xnview.exe output.tga

$(DESTDIR)$(TARGET): $(OBJECTS)
	$(SYSCONF_LINK) -Wall $(LDFLAGS) -o $(DESTDIR)$(TARGET) $(OBJECTS) $(LIBS)

$(OBJECTS): %.o: %.cpp
	$(SYSCONF_LINK) -Wall $(CPPFLAGS) -c $(CFLAGS) $< -o $@

clean:
	-$(RM) $(OBJECTS)
	-$(RM) $(TARGET)
	-$(RM) *.tga
	-$(RM) *.exe