import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import Forklift.Core
import Forklift.Gui

// Üst çubuk: ROS / SİM / SAĞLIK özetleri, sim zamanı, RTF, kayıt göstergesi,
// yardım ve acil durdurma (ARCHITECTURE §7.10).
Rectangle {
    id: bar

    // Sim zamanı → "SS:DD:ss.s" (yalnız biçim).
    function formatTime(seconds: real): string {
        const total = Math.max(0, seconds);
        const h = Math.floor(total / 3600);
        const m = Math.floor((total % 3600) / 60);
        const s = total - h * 3600 - m * 60;
        return String(h).padStart(2, "0") + ":" + String(m).padStart(2, "0") + ":" + s.toFixed(1).padStart(4, "0");
    }

    color: Theme.panelHeader

    Rectangle {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        height: 1
        color: Theme.border
    }

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: Theme.margin + Theme.estopFrameWidth
        anchors.rightMargin: Theme.margin + Theme.estopFrameWidth
        spacing: Theme.spacingLg + Theme.spacing

        StatusLed {
            label: Backend.mockMode ? qsTr("MOCK") : qsTr("ROS")
            status: Backend.health.rosState
            ledColor: Backend.mockMode ? Theme.info : Theme.stateColor(Backend.health.rosState)
        }

        StatusLed {
            label: qsTr("SİM")
            status: Backend.health.simState
        }

        Row {
            spacing: Theme.spacingSm

            Text {
                anchors.baseline: simTime.baseline
                text: qsTr("sim")
                color: Theme.textDim
                font.pixelSize: Theme.fontSm
            }

            Text {
                id: simTime

                text: Backend.telemetry.hasClock ? bar.formatTime(Backend.telemetry.simTime) : "--:--:--.-"
                color: Theme.text
                font.family: Theme.monoFamily
                font.pixelSize: Theme.fontLg
            }
        }

        Row {
            spacing: Theme.spacingSm

            Text {
                anchors.baseline: rtf.baseline
                text: qsTr("RTF")
                color: Theme.textDim
                font.pixelSize: Theme.fontSm
            }

            Text {
                id: rtf

                text: Backend.telemetry.rtfValid ? Backend.telemetry.rtf.toFixed(2) : "—.——"
                color: Backend.telemetry.rtfValid && Backend.telemetry.rtf < 0.9 ? Theme.warn : Theme.text
                font.family: Theme.monoFamily
                font.pixelSize: Theme.fontLg
            }
        }

        StatusLed {
            label: qsTr("SAĞLIK")
            status: Backend.health.overallState
        }

        // Sürekli kayıt görünür kalsın (Veri seti sekmesi kapalıyken bile, D-37).
        Rectangle {
            Layout.preferredHeight: Theme.buttonHeight - 6
            Layout.preferredWidth: recText.implicitWidth + 2 * Theme.spacing
            radius: Theme.radius
            color: Theme.dangerDark
            visible: Backend.dataset.recording

            Text {
                id: recText

                anchors.centerIn: parent
                text: qsTr("● KAYIT %1").arg(Backend.dataset.captureCount)
                color: Theme.textOnDanger
                font.family: Theme.monoFamily
                font.pixelSize: Theme.fontSm
                font.bold: true
            }
        }

        Item {
            Layout.fillWidth: true
        }

        Button {
            text: qsTr("Yardım  [F1]")
            flat: true
            focusPolicy: Qt.NoFocus
            onClicked: Backend.helpVisible = !Backend.helpVisible
        }

        EStopButton {
            Layout.preferredWidth: Theme.estopButtonWidth
            Layout.preferredHeight: Theme.estopButtonHeight
        }
    }
}
