#
# Makefile for the Kinibi core and trusted UI drivers
#

ifneq (,$(findstring _O8,$(PRIVATE_RCMS_NAME)))
USE_TBASE := 400a
else
ifneq (,$(findstring _N7,$(PRIVATE_RCMS_NAME)))
USE_TBASE := 310d
else
USE_TBASE := 310a
endif
endif

obj-$(CONFIG_TRUSTONIC_TEE) := $(USE_TBASE)/MobiCoreDriver/
obj-$(CONFIG_TRUSTONIC_TRUSTED_UI) += $(USE_TBASE)/TlcTui/
