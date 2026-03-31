HOST_EXEEXT = $(findstring .exe,$(MAKE))
HOSTCC = $(LOCAL_TCPREFIX)gcc$(LOCAL_TCSUFFIX)$(HOST_EXEEXT)
HOSTCXX = $(LOCAL_TCPREFIX)g++$(LOCAL_TCSUFFIX)$(HOST_EXEEXT)

# Filter out any include paths that point into the output directory,
# because those contain target-specific headers which may conflict with
# host system headers (e.g. musl vs glibc).
# We must use a more specific pattern to avoid filtering out necessary
# project headers if they were somehow absolute or started with output
# but were not what we wanted to filter.
# Actually, the previous filter-out was too broad because it matched anything
# with "output" in it if it wasn't carefully anchored.
HOST_CPPFLAGS = $(filter-out -I$(topdir)/output% -isystem $(topdir)/output% -Ioutput% -isystem output% -I$(OUT)% -isystem $(OUT)%,$(TARGET_INCLUDES) $(INCLUDES) $(TARGET_CPPFLAGS) $(CPPFLAGS))
HOST_CPPFLAGS := $(filter-out -I/opt/xcsoar/output% -isystem /opt/xcsoar/output%,$(HOST_CPPFLAGS))
HOST_CPPFLAGS += -I$(SRC) -I$(ENGINE_SRC_DIR) -I$(OUT)/include
HOST_CXXFLAGS = $(OPTIMIZE) $(HOST_OPTIMIZE) $(HOST_CXX_FEATURES) $(CXX_WARNINGS)

ifeq ($(HOST_IS_WIN32),y)
HOST_CPPFLAGS += -DHAVE_MSVCRT
endif

host-cc-flags = $(DEPFLAGS) $(HOST_CFLAGS) $(HOST_CPPFLAGS)
host-cxx-flags = $(DEPFLAGS) $(HOST_CXXFLAGS) $(HOST_CPPFLAGS)
host-ld-libs = -lm -lstdc++

WRAPPED_HOST_CC = $(CCACHE) $(HOSTCC)
WRAPPED_HOST_CXX = $(CCACHE) $(HOSTCXX)

.SECONDEXPANSION:

$(HOST_OUTPUT_DIR)/%.o: %.c | $$(dir $$@)dirstamp
	@$(NQ)echo "  HOSTCC  $@"
	$(Q)$(WRAPPED_HOST_CC) -c $(host-cc-flags) -o $@ $^

$(HOST_OUTPUT_DIR)/%.o: %.cpp | $$(dir $$@)dirstamp
	@$(NQ)echo "  HOSTCXX $@"
	$(Q)$(WRAPPED_HOST_CXX) -c $(host-cxx-flags) -o $@ $^

$(HOST_OUTPUT_DIR)/%$(HOST_EXEEXT): $(HOST_OUTPUT_DIR)/%.o
	@$(NQ)echo "  HOSTLD  $@"
	$(Q)$(WRAPPED_HOST_CC) $^ $(host-ld-libs) -o $@
