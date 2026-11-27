# Paths are relative to src/
INET_PROJ = ../../inet4.5
FLORA_PROJ = ../../flora-1.2.0

all: checkmakefiles
	@cd src && $(MAKE)

clean: checkmakefiles
	@cd src && $(MAKE) clean

cleanall: checkmakefiles
	@cd src && $(MAKE) MODE=release clean
	@cd src && $(MAKE) MODE=debug clean
	@rm -f src/Makefile

makefiles:
	@cd src && opp_makemake --make-so -o lorav2v -O out -f --deep \
	    -KINET_PROJ=$(INET_PROJ) -KFLORA_PROJ=$(FLORA_PROJ) -DINET_IMPORT \
	    -I. -I$$\(INET_PROJ\)/src -I$$\(FLORA_PROJ\)/src \
	    -L$$\(INET_PROJ\)/src -L$$\(FLORA_PROJ\)/src -lINET$$\(D\) -lflora$$\(D\)

checkmakefiles:
	@if [ ! -f src/Makefile ]; then \
	echo; \
	echo '======================================================================='; \
	echo 'src/Makefile does not exist. Please use "make makefiles" to generate it!'; \
	echo '======================================================================='; \
	echo; \
	exit 1; \
	fi
