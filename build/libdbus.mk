ifeq ($(TARGET_IS_LINUX),y)
ifeq ($(USE_POLL_EVENT),y)
ifneq ($(filter y,$(TARGET_IS_KOBO) $(TARGET_IS_COLIBRI)),y)

$(eval $(call pkg-config-library,LIBDBUS,dbus-1))

DBUS_SOURCES = \
	$(SRC)/lib/dbus/Connection.cxx \
	$(SRC)/lib/dbus/Error.cxx \
	$(SRC)/lib/dbus/Message.cxx \
	$(SRC)/lib/dbus/ScopeMatch.cxx \
	$(SRC)/lib/dbus/TimeDate.cxx \
	$(SRC)/lib/dbus/Systemd.cxx

DBUS_CPPFLAGS = $(LIBDBUS_CPPFLAGS)
INCLUDES += $(DBUS_CPPFLAGS)

$(eval $(call link-library,dbus,DBUS))

DBUS_LDLIBS += $(LIBDBUS_LDLIBS)

endif
endif
endif