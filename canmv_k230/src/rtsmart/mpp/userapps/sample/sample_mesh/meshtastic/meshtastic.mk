MESHTASTIC_PATH ?= ${shell pwd}/meshtastic

MESHTASTIC_CSRCS += $(shell find $(MESHTASTIC_PATH)/src -type f -name '*.c')

CXXEXT := .cpp
MESHTASTIC_CXXSRCS += $(shell find $(MESHTASTIC_PATH)/src -type f -name '*${CXXEXT}')

MESHTASTIC_CFLAGS += -I$(MESHTASTIC_PATH)
MESHTASTIC_CFLAGS += -I${shell pwd}/meshtastic/src
MESHTASTIC_CFLAGS += -I${shell pwd}/meshtastic/src/mesh
MESHTASTIC_CFLAGS += -I${shell pwd}/meshtastic/src/mesh/generated
MESHTASTIC_CXXFLAGS += -I$(MESHTASTIC_PATH)
MESHTASTIC_CXXFLAGS += -I${shell pwd}/meshtastic/src
MESHTASTIC_CXXFLAGS += -I${shell pwd}/meshtastic/src/mesh
MESHTASTIC_CXXFLAGS += -I${shell pwd}/meshtastic/src/mesh/generated
