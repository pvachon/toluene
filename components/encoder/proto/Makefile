PROTOS=ble_device.proto
TARGET_DIR=.

PBSRC=$(PROTOS:.proto=.pb-c.c)
PBGEN=$(PROTOS:.proto=.pb-c.h) $(PBSRC)

# Keep the output files around please
.PRECIOUS: $(PBGEN)

all: $(TARGET_DIR) $(PBSRC)

%.pb-c.c: %.proto
	@protoc-c --c_out=$(TARGET_DIR) $(PROTOS)

clean:
	$(RM) $(PBGEN)

.PHONY: clean all
