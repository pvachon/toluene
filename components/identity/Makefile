OBJ=identity.o \
	crypto.o \
	$(patsubst %c,%o,$(wildcard asn1/*.c))

OFLAGS=-O0 -ggdb
DEFINES=
INCLUDES=-I. -Iasn1/
HFLAGS=-MMD -MP
CXXFLAGS=-std=c++14 -g $(INCLUDES) -Wall -Wextra $(DEFINES) $(OFLAGS) $(HFLAGS)
CFLAGS=$(INCLUDES) -Wall -Wextra $(DEFINES) $(OFLAGS) $(HFLAGS)

LIBS=-lboost_program_options -lboost_system -lcryptopp
LDFLAGS=$(LIBS)

inc=$(OBJ:%.o=%.d)

TARGET=identity

-include $(inc)

$(TARGET): $(OBJ)
	$(CXX) -o $(TARGET) $(OBJ) $(LDFLAGS)

clean:
	$(RM) $(TARGET)
	$(RM) $(OBJ)
	$(RM) $(inc)

.PHONY: clean

