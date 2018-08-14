CC = gcc 

SRC_DIR=examples
BIN_DIR=bin

#CFLAGS += -I$(PULP_SDK_HOME)/install/include -D__OPENRISC__ -D_LITTLE_ENDIAN_ -Wno-incompatible-pointer-types
LDFLAGS += -lopenvx -lopenvx-helper -lvx_debug -lopenvx-debug_k-lib -lopenvx-debug-lib -lopenvx-extras-lib  -lm -Wl,--no-whole-archive


all: $(SRC_DIR)/sobel.c
	mkdir -p $(BIN_DIR)
	$(CC) -o $(BIN_DIR)/sobel $(SRC_DIR)/sobel.c  $(CFLAGS)  $(LDFLAGS)
	$(CC) -o $(BIN_DIR)/pyramid $(SRC_DIR)/pyramid.c  $(CFLAGS)  $(LDFLAGS)
	$(CC) -o $(BIN_DIR)/harris2  $(SRC_DIR)/harris2.c $(SRC_DIR)/vx_unibo_ext.c  $(CFLAGS)  $(LDFLAGS)
	$(CC) -o $(BIN_DIR)/canny  $(SRC_DIR)/canny.c  $(CFLAGS)  $(LDFLAGS)
	$(CC) -o $(BIN_DIR)/optical  $(SRC_DIR)/optical.c  $(CFLAGS)  $(LDFLAGS)

.PHONY: clean

clean:
	rm -rf $(BIN_DIR)/*
