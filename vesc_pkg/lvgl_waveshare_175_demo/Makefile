VESC_TOOL ?= vesc_tool

all: lvgl_waveshare_175_demo.vescpkg

lvgl_waveshare_175_demo.vescpkg: pkgdesc.qml README.md main.lisp ui.qml \
	fonts/eurostile-100.bin fonts/eurostile-40.bin \
	fonts/eurostile-30.bin fonts/eurostile-20.bin \
	assets/battery.lvim assets/controller.lvim assets/motor.lvim \
	assets/eco.lvim assets/race.lvim assets/blank.lvim
	$(VESC_TOOL) --buildPkgFromDesc pkgdesc.qml \
		--testPkgDesc "custom:waveshare amoled 1.75" \
		--testPkgDesc "custom:test" --testPkgDesc "vesc:test"

clean:
ifeq ($(OS),Windows_NT)
	powershell -NoProfile -Command "Remove-Item -Force -ErrorAction SilentlyContinue -LiteralPath 'lvgl_waveshare_175_demo.vescpkg'; exit 0"
else
	rm -f lvgl_waveshare_175_demo.vescpkg
endif

.PHONY: all clean
