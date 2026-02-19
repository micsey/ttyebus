# Makefile für ttyebus Kernel-Modul (Raspberry Pi 3/4 - Bookworm)

# Name des Moduls
obj-m += ttyebus.o
ccflags-y += -Wno-declaration-after-statement

# Pfade
TARGET_MODULE := ttyebus
BUILDSYSTEM_DIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
CONFIG_FILE := config.h

# Standard-Target: Kompilieren
all: $(CONFIG_FILE)
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) modules KBUILD_MODPOST_WARN=1 KBUILD_MODPOST_NO_MISSING_OK=1

# Prüft, ob das configure-Skript ausgeführt wurde
$(CONFIG_FILE):
	@if [ ! -f $(CONFIG_FILE) ]; then \
		echo "Fehler: $(CONFIG_FILE) fehlt!"; \
		echo "Bitte führen Sie zuerst './configure' aus."; \
		exit 1; \
	fi

# Hilfs-Target: Führt das configure-Skript automatisch aus
config:
	@chmod +x configure
	@./configure

# Installieren des Moduls
# Missing warning kann ignoriert werden!!!
install:
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) modules_install
	depmod -a
	modprobe $(TARGET_MODULE)
	sed -i "s/$(TARGET_MODULE)//g" /etc/modules
	echo "$(TARGET_MODULE)" >> /etc/modules

# Bereinigen
clean:
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) clean
	sed -i "s/$(TARGET_MODULE)//g" /etc/modules
	rm -f $(CONFIG_FILE)
	modprobe -r $(TARGET_MODULE)

# build-Verzeichnis bereinigen
distclean:
        rm -f $(TARGET_MODULE).o* $(TARGET_MODULE).mod* $(TARGET_MODULE).ko*
        rm -f $(CONFIG_FILE) .$(TARGET_MODULE)*

.PHONY: all config install clean distclean
