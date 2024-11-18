# invoke SourceDir generated makefile for rtos.pem4f
rtos.pem4f: .libraries,rtos.pem4f
.libraries,rtos.pem4f: package/cfg/rtos_pem4f.xdl
	$(MAKE) -f C:\Users\deote\OneDrive\Desktop\CCSWOR~1\ece3849_lab3_msimantirakis_tdmakoye/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\deote\OneDrive\Desktop\CCSWOR~1\ece3849_lab3_msimantirakis_tdmakoye/src/makefile.libs clean

