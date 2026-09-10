import Vedder.vesc.vescinterface 1.0
import Vedder.vesc.commands 1.0

import QtQuick 2.7
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.3

Item {
    id: root

    anchors.fill: parent
    Material.theme: Material.Dark
    Material.accent: Material.Orange

    readonly property int protocolMagic0: 0x56
    readonly property int protocolMagic1: 0x44
    readonly property int protocolVersion: 1
    readonly property int opcodeGet: 1
    readonly property int opcodeSet: 2
    readonly property int opcodeGetProfiles: 3
    readonly property int opcodeSaveProfile: 4
    readonly property int opcodeCommitProfile: 5
    readonly property var profileNames: ["Eco", "Sport", "Race"]

    property Commands mCommands: VescIf.commands()
    property int nextRequestId: 1
    property int pendingRequestId: 0
    property string pendingKind: ""
    property bool requestPending: false
    property bool settingsLoaded: false
    property bool profileDataLoaded: false
    property var profileRecords: []
    property int targetControllerId: 255
    property int editorSpeedUnit: 0
    property bool editorLoading: false
    property bool applyAfterProfileSave: false
    property int profileSaveIndex: 0
    property bool profileEnabled: false
    property bool profileSynced: false
    property string statusText: "Reading settings from the display..."
    property color statusColor: "#b7bdc8"
    readonly property bool busy: requestPending

    function allocateRequestId() {
        var id = nextRequestId
        nextRequestId = nextRequestId >= 255 ? 1 : nextRequestId + 1
        return id
    }

    function sendRequest(kind, buffer, message) {
        if (requestPending) {
            return false
        }

        var requestId = allocateRequestId()
        var data = new DataView(buffer)
        data.setUint8(0, protocolMagic0)
        data.setUint8(1, protocolMagic1)
        data.setUint8(2, protocolVersion)
        data.setUint8(4, requestId)
        pendingRequestId = requestId
        pendingKind = kind
        requestPending = true
        requestTimeout.interval = kind === "profile-commit" ? 5000 : 2500
        requestTimeout.restart()
        statusText = message
        statusColor = "#b7bdc8"
        mCommands.sendCustomAppData(buffer)
        return true
    }

    function requestSettings() {
        var buffer = new ArrayBuffer(5)
        var data = new DataView(buffer)
        data.setUint8(3, opcodeGet)
        sendRequest("settings-get", buffer, "Reading display settings...")
    }

    function saveSettings() {
        var buffer = new ArrayBuffer(8)
        var data = new DataView(buffer)
        data.setUint8(3, opcodeSet)
        data.setUint8(5, Math.round(brightnessSlider.value))
        data.setUint8(6, speedUnit.currentIndex)
        data.setUint8(7, powerUnit.currentIndex)
        sendRequest("settings-set", buffer, "Saving display settings...")
    }

    function requestProfiles() {
        var buffer = new ArrayBuffer(5)
        var data = new DataView(buffer)
        data.setUint8(3, opcodeGetProfiles)
        sendRequest("profiles-get", buffer, "Reading temporary power profiles...")
    }

    function canonicalSpeedX10(displayValue) {
        if (speedUnit.currentIndex === 1) {
            return Math.max(0, Math.min(3000,
                                        Math.round(displayValue / 0.621371)))
        }
        return Math.max(0, Math.min(3000, Math.round(displayValue)))
    }

    function displayedSpeedX10(canonicalValue) {
        return speedUnit.currentIndex === 1 ?
                    Math.round(canonicalValue * 0.621371) : canonicalValue
    }

    function editorRecord() {
        return {
            brakePct: brakeCurrent.value,
            drivePct: driveCurrent.value,
            reverseKmhX10: canonicalSpeedX10(reverseSpeed.value),
            forwardKmhX10: canonicalSpeedX10(forwardSpeed.value),
            minDutyPct: minDuty.value,
            maxDutyPct: maxDuty.value,
            regenWatts: regenWatts.value,
            driveWatts: driveWatts.value
        }
    }

    function loadProfileEditor(index) {
        if (!profileDataLoaded || index < 0 || index >= profileRecords.length) {
            return
        }
        var record = profileRecords[index]
        editorLoading = true
        brakeCurrent.value = record.brakePct
        driveCurrent.value = record.drivePct
        reverseSpeed.value = displayedSpeedX10(record.reverseKmhX10)
        forwardSpeed.value = displayedSpeedX10(record.forwardKmhX10)
        minDuty.value = record.minDutyPct
        maxDuty.value = record.maxDutyPct
        regenWatts.value = record.regenWatts
        driveWatts.value = record.driveWatts
        editorSpeedUnit = speedUnit.currentIndex
        editorLoading = false
    }

    function convertEditorSpeedUnit(newUnit) {
        if (!profileDataLoaded || editorLoading || newUnit === editorSpeedUnit) {
            editorSpeedUnit = newUnit
            return
        }

        var factor = newUnit === 1 ? 0.621371 : (1.0 / 0.621371)
        editorLoading = true
        reverseSpeed.value = Math.round(reverseSpeed.value * factor)
        forwardSpeed.value = Math.round(forwardSpeed.value * factor)
        editorSpeedUnit = newUnit
        editorLoading = false
    }

    function saveAndApplyProfile() {
        if (!profileDataLoaded || targetControllerId === 255) {
            statusText = "No live VESC controller is visible to the display. Start the controller and refresh profiles."
            statusColor = "#ff9b85"
            return
        }
        if (minDuty.value > maxDuty.value) {
            statusText = "Minimum duty cannot be greater than maximum duty."
            statusColor = "#ff9b85"
            return
        }

        var profile = profileSelector.currentIndex
        var record = editorRecord()
        var buffer = new ArrayBuffer(18)
        var data = new DataView(buffer)
        data.setUint8(3, opcodeSaveProfile)
        data.setUint8(5, profile)
        data.setUint8(6, record.brakePct)
        data.setUint8(7, record.drivePct)
        data.setUint16(8, record.reverseKmhX10)
        data.setUint16(10, record.forwardKmhX10)
        data.setUint8(12, record.minDutyPct)
        data.setUint8(13, record.maxDutyPct)
        data.setUint16(14, record.regenWatts)
        data.setUint16(16, record.driveWatts)
        profileSaveIndex = profile
        applyAfterProfileSave = true
        if (!sendRequest("profile-save", buffer,
                         "Saving " + profileNames[profile] + " profile...")) {
            applyAfterProfileSave = false
        }
    }

    function sendProfileCommit() {
        var buffer = new ArrayBuffer(6)
        var data = new DataView(buffer)
        data.setUint8(3, opcodeCommitProfile)
        data.setUint8(5, profileSaveIndex)
        sendRequest("profile-commit", buffer,
                    "Express is applying " + profileNames[profileSaveIndex] +
                    " and waiting for the controller acknowledgement...")
    }

    function parseProfiles(view) {
        var parsed = []
        targetControllerId = view.getUint8(6)
        var selected = Math.min(2, view.getUint8(7))
        for (var profile = 0; profile < 3; profile++) {
            var offset = 8 + profile * 12
            parsed.push({
                brakePct: view.getUint8(offset),
                drivePct: view.getUint8(offset + 1),
                reverseKmhX10: view.getUint16(offset + 2),
                forwardKmhX10: view.getUint16(offset + 4),
                minDutyPct: view.getUint8(offset + 6),
                maxDutyPct: view.getUint8(offset + 7),
                regenWatts: view.getUint16(offset + 8),
                driveWatts: view.getUint16(offset + 10)
            })
        }
        profileRecords = parsed
        profileEnabled = view.byteLength >= 46 && view.getUint8(44) === 1
        profileSynced = view.byteLength >= 46 && view.getUint8(45) === 1
        profileDataLoaded = true
        editorLoading = true
        profileSelector.currentIndex = selected
        editorLoading = false
        loadProfileEditor(selected)
    }

    Component.onCompleted: startupRequest.start()

    Rectangle {
        anchors.fill: parent
        color: "#101214"
    }

    ScrollView {
        id: settingsView
        anchors.fill: parent
        contentWidth: availableWidth
        clip: true

        ColumnLayout {
            width: Math.max(280, settingsView.availableWidth - 32)
            x: 16
            spacing: 16

            Item { Layout.preferredHeight: 4 }

            Label {
                text: "VESC Display"
                color: "#ffffff"
                font.pixelSize: 28
                font.bold: true
                Layout.fillWidth: true
            }

            Label {
                text: "Waveshare AMOLED 1.75 settings"
                color: "#9ca3af"
                font.pixelSize: 15
                Layout.fillWidth: true
            }

            Frame {
                Layout.fillWidth: true
                padding: 18
                background: Rectangle {
                    color: "#191d21"
                    border.color: "#30363d"
                    border.width: 1
                    radius: 10
                }

                ColumnLayout {
                    width: parent.width
                    spacing: 10

                    RowLayout {
                        Layout.fillWidth: true
                        Label {
                            text: "Brightness"
                            color: "#ffffff"
                            font.pixelSize: 18
                            font.bold: true
                        }
                        Item { Layout.fillWidth: true }
                        Label {
                            text: Math.round(brightnessSlider.value) + "%"
                            color: Material.accent
                            font.pixelSize: 18
                            font.bold: true
                        }
                    }

                    Slider {
                        id: brightnessSlider
                        Layout.fillWidth: true
                        from: 1
                        to: 100
                        stepSize: 1
                        value: 80
                        enabled: settingsLoaded && !busy
                    }

                    Label {
                        text: "Applied by the AMOLED controller and restored at startup."
                        color: "#9ca3af"
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                    }
                }
            }

            Frame {
                Layout.fillWidth: true
                padding: 18
                background: Rectangle {
                    color: "#191d21"
                    border.color: "#30363d"
                    border.width: 1
                    radius: 10
                }

                ColumnLayout {
                    width: parent.width
                    spacing: 14

                    Label {
                        text: "Dashboard units"
                        color: "#ffffff"
                        font.pixelSize: 18
                        font.bold: true
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 12
                        Label {
                            text: "Speed"
                            color: "#d6dae0"
                            Layout.preferredWidth: 110
                        }
                        ComboBox {
                            id: speedUnit
                            model: ["km/h", "mph"]
                            Layout.fillWidth: true
                            enabled: settingsLoaded && !busy
                            onCurrentIndexChanged: convertEditorSpeedUnit(currentIndex)
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 12
                        Label {
                            text: "Power"
                            color: "#d6dae0"
                            Layout.preferredWidth: 110
                        }
                        ComboBox {
                            id: powerUnit
                            model: ["Watts", "Battery amps"]
                            Layout.fillWidth: true
                            enabled: settingsLoaded && !busy
                        }
                    }

                    Label {
                        text: "Battery amps uses VESC input current, including negative regenerative current."
                        color: "#9ca3af"
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                    }
                }
            }

            Frame {
                Layout.fillWidth: true
                padding: 18
                background: Rectangle {
                    color: "#191d21"
                    border.color: "#30363d"
                    border.width: 1
                    radius: 10
                }

                ColumnLayout {
                    width: parent.width
                    spacing: 12

                    RowLayout {
                        Layout.fillWidth: true
                        Label {
                            text: "Temporary power profiles"
                            color: "#ffffff"
                            font.pixelSize: 18
                            font.bold: true
                        }
                        Item { Layout.fillWidth: true }
                        Label {
                            text: targetControllerId === 255 ? "CAN --" :
                                  "CAN " + targetControllerId +
                                  (profileSynced ? "  Synced" :
                                   (profileEnabled ? "  Pending" : "  Inactive"))
                            color: targetControllerId === 255 ? "#ff9b85" :
                                   (profileSynced ? "#78dba9" :
                                    (profileEnabled ? "#ffbf69" : "#9ca3af"))
                            font.bold: true
                        }
                    }

                    ComboBox {
                        id: profileSelector
                        model: profileNames
                        Layout.fillWidth: true
                        enabled: profileDataLoaded && !busy
                        onCurrentIndexChanged: {
                            if (!editorLoading) {
                                loadProfileEditor(currentIndex)
                            }
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "Drive current"; color: "#d6dae0"; Layout.fillWidth: true }
                        SpinBox {
                            id: driveCurrent
                            from: 0; to: 100; value: 100; editable: true
                            enabled: profileDataLoaded && !busy
                        }
                        Label { text: "%"; color: "#9ca3af"; Layout.preferredWidth: 42 }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "Brake current"; color: "#d6dae0"; Layout.fillWidth: true }
                        SpinBox {
                            id: brakeCurrent
                            from: 0; to: 100; value: 100; editable: true
                            enabled: profileDataLoaded && !busy
                        }
                        Label { text: "%"; color: "#9ca3af"; Layout.preferredWidth: 42 }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "Forward speed"; color: "#d6dae0"; Layout.fillWidth: true }
                        SpinBox {
                            id: forwardSpeed
                            from: 0; to: 3000; value: 990; stepSize: 10; editable: true
                            enabled: profileDataLoaded && !busy
                            textFromValue: function(value) { return (value / 10.0).toFixed(1) }
                            valueFromText: function(text) { return Math.round(parseFloat(text.replace(",", ".")) * 10.0) }
                        }
                        Label { text: speedUnit.currentIndex === 1 ? "mph" : "km/h"; color: "#9ca3af"; Layout.preferredWidth: 42 }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "Reverse speed"; color: "#d6dae0"; Layout.fillWidth: true }
                        SpinBox {
                            id: reverseSpeed
                            from: 0; to: 3000; value: 990; stepSize: 10; editable: true
                            enabled: profileDataLoaded && !busy
                            textFromValue: function(value) { return (value / 10.0).toFixed(1) }
                            valueFromText: function(text) { return Math.round(parseFloat(text.replace(",", ".")) * 10.0) }
                        }
                        Label { text: speedUnit.currentIndex === 1 ? "mph" : "km/h"; color: "#9ca3af"; Layout.preferredWidth: 42 }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "Drive power"; color: "#d6dae0"; Layout.fillWidth: true }
                        SpinBox {
                            id: driveWatts
                            from: 0; to: 30000; value: 9999; stepSize: 100; editable: true
                            enabled: profileDataLoaded && !busy
                        }
                        Label { text: "W"; color: "#9ca3af"; Layout.preferredWidth: 42 }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "Regen power"; color: "#d6dae0"; Layout.fillWidth: true }
                        SpinBox {
                            id: regenWatts
                            from: 0; to: 30000; value: 9999; stepSize: 100; editable: true
                            enabled: profileDataLoaded && !busy
                        }
                        Label { text: "W"; color: "#9ca3af"; Layout.preferredWidth: 42 }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "Minimum duty"; color: "#d6dae0"; Layout.fillWidth: true }
                        SpinBox {
                            id: minDuty
                            from: 0; to: 100; value: 5; editable: true
                            enabled: profileDataLoaded && !busy
                        }
                        Label { text: "%"; color: "#9ca3af"; Layout.preferredWidth: 42 }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "Maximum duty"; color: "#d6dae0"; Layout.fillWidth: true }
                        SpinBox {
                            id: maxDuty
                            from: 0; to: 100; value: 95; editable: true
                            enabled: profileDataLoaded && !busy
                        }
                        Label { text: "%"; color: "#9ca3af"; Layout.preferredWidth: 42 }
                    }

                    Label {
                        text: "Save & Apply stores the profile on Express and applies it to controller RAM only. Express reapplies it after a restart and verifies alignment every 15 seconds without writing motor configuration flash."
                        color: "#9ca3af"
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 12
                        Button {
                            text: "Refresh profiles"
                            enabled: !busy
                            Layout.fillWidth: true
                            onClicked: requestProfiles()
                        }
                        Button {
                            text: "Save & Apply"
                            highlighted: true
                            enabled: profileDataLoaded && !busy
                            Layout.fillWidth: true
                            onClicked: saveAndApplyProfile()
                        }
                    }
                }
            }

            Label {
                text: statusText
                color: statusColor
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
                Layout.fillWidth: true
                Layout.minimumHeight: 24
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 12
                Button {
                    text: "Refresh display"
                    enabled: !busy
                    Layout.fillWidth: true
                    onClicked: requestSettings()
                }
                Button {
                    text: "Apply display"
                    highlighted: true
                    enabled: settingsLoaded && !busy
                    Layout.fillWidth: true
                    onClicked: saveSettings()
                }
            }

            Label {
                text: "The selected profile is enforced by the display while the controller is online. Its icon is shown only while synchronization is verified."
                color: "#737b87"
                horizontalAlignment: Text.AlignHCenter
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            Item { Layout.preferredHeight: 8 }
        }
    }

    Timer {
        id: startupRequest
        interval: 250
        repeat: false
        onTriggered: requestSettings()
    }

    Timer {
        id: requestTimeout
        interval: 2500
        repeat: false
        onTriggered: {
            var failedKind = pendingKind
            requestPending = false
            pendingKind = ""
            applyAfterProfileSave = false
            if (failedKind === "profile-commit") {
                profileSynced = false
            }
            statusText = failedKind === "profile-commit" ?
                        "Express did not receive the controller acknowledgement. The profile was saved but is not marked synchronized." :
                        "No response. Connect directly to the display and try again."
            statusColor = "#ff9b85"
        }
    }

    Timer {
        id: profileRetry
        interval: 500
        repeat: false
        onTriggered: requestProfiles()
    }

    Connections {
        target: mCommands

        function onCustomAppDataReceived(data) {
            if (data.byteLength < 6 || !requestPending) {
                return
            }

            var view = new DataView(data)
            if (view.getUint8(0) !== protocolMagic0 ||
                    view.getUint8(1) !== protocolMagic1 ||
                    view.getUint8(2) !== protocolVersion ||
                    view.getUint8(4) !== pendingRequestId) {
                return
            }

            var opcode = view.getUint8(3)
            var completedKind = pendingKind
            var expected = (completedKind === "settings-get" && opcode === 0x81) ||
                    (completedKind === "settings-set" && opcode === 0x82) ||
                    (completedKind === "profiles-get" && opcode === 0x83) ||
                    (completedKind === "profile-save" && opcode === 0x84) ||
                    (completedKind === "profile-commit" && opcode === 0x85)
            if (!expected) {
                return
            }

            requestTimeout.stop()
            requestPending = false
            pendingKind = ""
            if (view.getUint8(5) !== 0) {
                applyAfterProfileSave = false
                if (completedKind === "profiles-get" && view.getUint8(5) === 5) {
                    statusText = "The physical dashboard is running; waiting for the profile guardian..."
                    statusColor = "#b7bdc8"
                    profileRetry.restart()
                    return
                }
                if (completedKind === "profile-commit") {
                    profileSynced = false
                    if (view.getUint8(5) === 3) {
                        statusText = "The controller did not acknowledge the temporary profile. Express will not show its icon."
                    } else if (view.getUint8(5) === 4) {
                        statusText = "The profile guardian is busy. Try Save & Apply again in a moment."
                    } else {
                        statusText = "The profile guardian is not ready (status " +
                                view.getUint8(5) + ")."
                    }
                    statusColor = "#ff9b85"
                    return
                }
                statusText = "The display rejected the request (status " +
                        view.getUint8(5) + ")."
                statusColor = "#ff9b85"
                return
            }

            if (opcode === 0x81 || opcode === 0x82) {
                if (data.byteLength < 9) {
                    statusText = "The display returned a short settings response."
                    statusColor = "#ff9b85"
                    return
                }
                brightnessSlider.value = view.getUint8(6)
                speedUnit.currentIndex = view.getUint8(7) === 1 ? 1 : 0
                powerUnit.currentIndex = view.getUint8(8) === 1 ? 1 : 0
                settingsLoaded = true
                statusText = opcode === 0x82 ?
                            "Display settings saved and applied." : "Display settings loaded."
                statusColor = "#78dba9"
                if (!profileDataLoaded) {
                    Qt.callLater(requestProfiles)
                }
                return
            }

            if (data.byteLength < 44) {
                applyAfterProfileSave = false
                statusText = "The display returned a short profile response."
                statusColor = "#ff9b85"
                return
            }
            parseProfiles(view)

            if (opcode === 0x84 && applyAfterProfileSave) {
                applyAfterProfileSave = false
                Qt.callLater(sendProfileCommit)
            } else if (opcode === 0x85) {
                statusText = profileNames[profileSaveIndex] +
                        " is synchronized. Express will restore it automatically after a controller restart."
                statusColor = "#78dba9"
            } else {
                statusText = "Temporary power profiles loaded."
                statusColor = "#78dba9"
            }
        }
    }
}
