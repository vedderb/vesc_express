import QtQuick 2.15

Item {
    property string pkgName: "LVGL Waveshare 1.75 Dashboard"
    property string pkgDescriptionMd: "README.md"
    property string pkgLisp: "main.lisp"
    property string pkgQml: "ui.qml"
    property bool pkgQmlIsFullscreen: false
    property string pkgOutput: "lvgl_waveshare_175_demo.vescpkg"

    function isCompatible(fwRxParams) {
        return fwRxParams.hwTypeStr().toLowerCase() === "custom module" &&
                fwRxParams.hw.toLowerCase() === "waveshare amoled 1.75"
    }
}
