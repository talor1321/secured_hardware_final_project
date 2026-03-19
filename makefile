# what the final build file will be called
TARGET = output

# our C firmware file
SRC += ${SOURCE}

# Use simpleserial V1
SS_VER = SS_VER_1_1

EXTRA_OPTS = NO_EXTRA_OPTS
CFLAGS += -D$(EXTRA_OPTS)

ifeq ($(CRYPTO_TARGET),)
  ${info No CRYPTO_TARGET passed - defaulting to TINYAES128C}
  CRYPTO_TARGET = TINYAES128C
endif

${info Building for platform ${PLATFORM} with CRYPTO_TARGET=$(CRYPTO_TARGET)}

#Add simpleserial project to build
include ../../firmware/mcu/simpleserial/Makefile.simpleserial

FIRMWAREPATH = ../../firmware/mcu
include $(FIRMWAREPATH)/Makefile.inc