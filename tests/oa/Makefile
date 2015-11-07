TARGET = main

# repertoire des modules
#AVERSIVE_DIR = ../../../libs/aversive_b_zer0_9oct09
AVERSIVE_DIR = ../../libs/aversive4dspic

SRC  = $(TARGET).c

ASRC =

CFLAGS += -Wall -DHOST_VERSION_OA_TEST -Wno-unused-but-set-variable -Wno-unused-value -Wno-unused-result


########################################

-include .aversive_conf
include $(AVERSIVE_DIR)/mk/aversive_project.mk
