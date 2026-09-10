import Vedder.vesc.codeloader 1.0

import QtQuick 2.7

Item {
    CodeLoader {
        id: loader
    }

    Timer {
        id: connectionPoll
        interval: 250
        repeat: true
        running: true

        onTriggered: {
            if (!VescIf.isPortConnected()) {
                return
            }

            stop()
            loader.setVesc(VescIf)
            var ok = loader.installVescPackageFromPath(
                        "D:/vesc express lvgl/vesc_pkg/lvgl_waveshare_175_demo/lvgl_waveshare_175_demo.vescpkg")
            console.log("PACKAGE_INSTALL_RESULT=" + (ok ? "OK" : "FAILED"))
            quitTimer.start()
        }
    }

    Timer {
        id: quitTimer
        interval: 750
        repeat: false
        onTriggered: Qt.quit()
    }

    Timer {
        interval: 15000
        repeat: false
        running: true
        onTriggered: {
            console.log("PACKAGE_INSTALL_RESULT=TIMEOUT")
            Qt.quit()
        }
    }
}
