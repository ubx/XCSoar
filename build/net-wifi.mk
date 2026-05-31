# Shared WiFi error formatting is used by WifiDialog on all platforms.
XCSOAR_SOURCES += \
	$(SRC)/net/wifi/WifiError.cpp

# Kobo/Colibri WiFi is provided by the wpa_supplicant backend.
ifneq ($(filter y,$(TARGET_IS_KOBO) $(TARGET_IS_COLIBRI)),)
XCSOAR_SOURCES += \
	$(SRC)/Kobo/System.cpp \
	$(SRC)/Kobo/WPASupplicant.cpp \
	$(SRC)/Kobo/WPASupplicantBackend.cpp \
	$(SRC)/Kobo/PlatformWifiBackend.cpp
endif

# NetworkManager / ConnMan settings via D-Bus (Linux only; not Kobo, not Android, not Colibri)
HAVE_LINUX_NET_WIFI := n
ifeq ($(TARGET_IS_LINUX),y)
ifeq ($(TARGET_IS_KOBO),n)
ifeq ($(TARGET_IS_COLIBRI),n)
ifeq ($(TARGET_IS_ANDROID),n)
HAVE_LINUX_NET_WIFI := y
endif
endif
endif
endif

ifeq ($(HAVE_LINUX_NET_WIFI),y)
XCSOAR_SOURCES += \
	$(SRC)/net/wifi/LinuxNetWifiDbus.cpp \
	$(SRC)/net/wifi/LinuxWifiBackend.cpp \
	$(SRC)/net/wifi/NetworkManagerClient.cpp \
	$(SRC)/net/wifi/NetworkManagerWifiBackend.cpp \
	$(SRC)/net/wifi/ConnmanClient.cpp \
	$(SRC)/net/wifi/ConnmanWifiBackend.cpp

XCSOAR_CPPFLAGS += -DHAVE_LINUX_NET_WIFI
endif
