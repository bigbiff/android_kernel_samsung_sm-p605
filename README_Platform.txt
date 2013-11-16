How to build Module for Platform
- It is only for modules are needed to using Android build system.
- Please check its own install information under its folder for other module.

[Step to build]
1. Get android open source.
	: version info - Android 4.3
	( Download site : http://source.android.com )

2. Copy module that you want to build to original android open source. If same module exist in android open source, you should replace it. (no overwrite)
	# It is possible to build all modules at once.

3. You should add module name to 'PRODUCT_PACKAGES' in 'build\target\product\core.mk' as following case.
	case 1) e2fsprog : should add 'e2fsck' to PRODUCT_PACKAGES
	case 2) KeyUtils : should add 'libkeyutils' to PRODUCT_PACKAGES
	case 3) libexifa : should add 'libexifa' to PRODUCT_PACKAGES
	case 4) libjpega : should add 'libjpega' to PRODUCT_PACKAGES
	case 5) alsa-lib : should add 'libalsa-intf' to PRODUCT_PACKAGES
	case 6) jack audio connection kit : shoud add belows to PRODUCT_PACKAGES
		libjackshm \
		libjackserver \
		libjack \
		androidshmservice \
		jackd \
		jack_dummy \
		jack_alsa \
		jack_loopback \
		jack_connect \
		jack_disconnect \
		jack_lsp \
		jack_showtime \
		jack_simple_client \
		jack_transport \
		jack_goldfish \
		libasound
	case 7) fluidsynth : shoud add belows to PRODUCT_PACKAGES
		libglib-2.0 \
		libgthread-2.0 \
		libfluidsynth

	ex.) [build\target\product\core.mk] - add all module name for case 1 ~ 7 at once
		PRODUCT_PACKAGES += \
			libjackshm \
			libjackserver \
			libjack \
			androidshmservice \
			jackd \
			jack_dummy \
			jack_alsa \
			jack_loopback \
			jack_connect \
			jack_disconnect \
			jack_lsp \
			jack_showtime \
			jack_simple_client \
			jack_transport \
			jack_goldfish \
			libasound

		PRODUCT_PACKAGES += \
			libglib-2.0 \
			libgthread-2.0 \
			libfluidsynth

		PRODUCT_PACKAGES += \
			e2fsck \
			libkeyutils \
			libexifa \
			libjpega \

4. You should add ALSA_AUDIO flag' in 'build\target\board\generic\BoardConfig.mk' as following case.
	BOARD_USES_ALSA_AUDIO=true

5. excute build command
	./build.sh user

* Note
To build SBrowser (vendor/samsung/common/packages/apps/SBrowser),
please refer to Buildme.txt at the folder mentioned above.