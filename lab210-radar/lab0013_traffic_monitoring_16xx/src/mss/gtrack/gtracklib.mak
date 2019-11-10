###################################################################################
# GTRACK Library Makefile
###################################################################################
.PHONY: gTrackLib gTrackLibClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src
vpath %.c platform

###################################################################################
# GTRACK Library Source Files:
###################################################################################
GTRACK_LIB_SOURCES = gtrack_create.c			\
					 gtrack_delete.c			\
					 gtrack_step.c				\
					 gtrack_module.c			\
					 gtrack_unit_create.c		\
					 gtrack_unit_delete.c		\
					 gtrack_unit_event.c		\
					 gtrack_unit_predict.c		\
					 gtrack_unit_report.c		\
					 gtrack_unit_score.c		\
					 gtrack_unit_start.c		\
					 gtrack_unit_stop.c			\
					 gtrack_unit_update.c		\
					 gtrack_unit_get.c			\
					 gtrack_utilities.c			\
					 gtrack_math.c				\
					 gtrack_listlib.c 			

###################################################################################
# Enabling Debug Support
###################################################################################
R4F_CFLAGS  += --define=GTRACK_LOG_ENABLED --define=GTRACK_ASSERT_ENABLED
C674_CFLAGS  += --define=GTRACK_LOG_ENABLED --define=GTRACK_ASSERT_ENABLED

###################################################################################
# GTRACK Library Source Files:
# - XWR14xx
#   GTRACK Library is available for the R4
# - XWR16xx:
#   GTRACK Library is available for the R4 and DSP
###################################################################################
ifeq ($(MMWAVE_SDK_DEVICE_TYPE),xwr14xx)
GTRACK_R4F_DRV_LIB_OBJECTS  = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(R4F_OBJ_EXT)))
GTRACK_C674_DRV_LIB_OBJECTS =
else
GTRACK_R4F_DRV_LIB_OBJECTS  = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(R4F_OBJ_EXT)))
GTRACK_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))
endif

###################################################################################
# Library Dependency:
###################################################################################
GTRACK_R4F_DRV_DEPENDS  = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(R4F_DEP_EXT)))
GTRACK_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(GTRACK_LIB_SOURCES:.c=.$(C674_DEP_EXT)))

###################################################################################
# GTRACK Library Names:
###################################################################################
GTRACK_R4F_DRV_LIB  = lib/libgtrack_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)
GTRACK_C674_DRV_LIB = lib/libgtrack_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)

R4F_CFLAGS += -i../ -i ../include

###################################################################################
# GTRACK Library Build:
# - XWR14xx: Build the R4 Library
# - XWR16xx: Build the R4 & DSP Library
###################################################################################
gTrackLib: buildDirectories $(GTRACK_R4F_DRV_LIB_OBJECTS) $(GTRACK_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(R4F_AR) $(R4F_AR_OPTS) $(GTRACK_R4F_DRV_LIB) $(GTRACK_R4F_DRV_LIB_OBJECTS)
ifeq ($(MMWAVE_SDK_DEVICE_TYPE),xwr16xx)
	$(C674_AR) $(C674_AR_OPTS) $(GTRACK_C674_DRV_LIB) $(GTRACK_C674_DRV_LIB_OBJECTS)
endif
###################################################################################
# Clean the GTRACK Libraries
###################################################################################
gTrackLibClean:
	@echo 'Cleaning the GTRACK Library Objects'
	@$(DEL) $(GTRACK_R4F_DRV_LIB_OBJECTS) $(GTRACK_R4F_DRV_LIB)
	@$(DEL) $(GTRACK_C674_DRV_LIB_OBJECTS) $(GTRACK_C674_DRV_LIB)
	@$(DEL) $(GTRACK_R4F_DRV_DEPENDS) $(GTRACK_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(GTRACK_R4F_DRV_DEPENDS)
-include $(GTRACK_C674_DRV_DEPENDS)

