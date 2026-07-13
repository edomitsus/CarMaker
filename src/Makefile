#******************************************************************************
#*  CarMaker - Version 15.0
#*  Virtual Test Driving
#*
#*  Copyright ©1998-2025 IPG Automotive GmbH. All rights reserved.
#*  www.ipg-automotive.com
#******************************************************************************
#**
#**  Application Makefile
#**  Run with 'make V=1 ...' for a verbose build.
#**
#******************************************************************************

include C:/IPG/carmaker/win64-15.0/include/MakeDefs.win64

APP_VER =		"Car_Generic <insert.your.version.no>"
APP_NAME =		CarMaker.$(ARCH)$(EXE_EXT)

#OPT_CFLAGS =		-g -O1

LD_LIBS =		$(CAR_LIB) \
			$(CARMAKER_LIB) $(DRIVER_LIB) $(ROAD_LIB) $(TIRE_LIB) $(MIO_LIB)
OBJS =			CM_Main.o CM_Vehicle.o User.o IO.o

# Prepend local include/library directory to include path:
# PREINC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib

# Append local include/library directory to include path:
# INC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib


### Linking with RTW-built Simulink models

#MATSUPP_MATVER =	R2022b
#LD_LIBS +=		$(MATSUPP_LIB)


# @@PLUGIN-BEGIN-LIBS@@ - Automatically generated code - don't edit!
# @@PLUGIN-END@@

### END (Linking with RTW-built Simulink models)


default:	$(APP_NAME)
$(APP_NAME):	$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK) app_tmp.o $(MINGW_DLLS)
	$(QECHO) " LD     $@"
	$Q $(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ \
		$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS) $(LDXX_LIBS_OS) \
		app_tmp.o
	$(SET_EXE_PERMISSIONS) $@


install: $(APP_NAME) $(MINGW_DLLS)
	$(INSTALL_APP) $(APP_NAME) $(ARCH)
	$Q cp -t ../bin $(MINGW_DLLS)

clean:
	-rm -f 	*~ *% *.o core

app_tmp.cpp: Makefile $(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK)
	$(QECHO) " MK     $@"
	$Q $(CREATE_INFO_CMD)

%.dll:	$(MINGW_LIB_DIR)/%.dll
	$Q cp $< .

depend .depend: Makefile
	$(QECHO) " MK     $@"
	@echo -n "" >.depend
ifneq ($(wildcard *.c),)
	$Q-$(CC)  $(CFLAGS)   $(DEPCFLAGS)   *.c   >>.depend 2>/dev/null
endif
ifneq ($(wildcard *.cpp),)
	$Q-$(CXX) $(CXXFLAGS) $(DEPCXXFLAGS) *.cpp >>.depend 2>/dev/null
endif
include .depend
