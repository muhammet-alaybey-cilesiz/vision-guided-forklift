import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import Forklift.Core
import Forklift.Gui

// Sürüş: sanal joystick, hız ölçeği ve komut / ölçülen hız (spesifikasyon §4.1).
PanelFrame {
    id: panel

    title: qsTr("Sürüş")
    headerExtras: [
        Text {
            text: Backend.teleop.active ? qsTr("● KOMUT GİDİYOR") : (Backend.teleop.enabled ? qsTr("hazır") : qsTr("devre dışı"))
            color: Backend.teleop.active ? Theme.ok : (Backend.teleop.enabled ? Theme.textDim : Theme.danger)
            font.pixelSize: Theme.fontXs
            font.bold: true
        }
    ]

    RowLayout {
        anchors.fill: parent
        spacing: Theme.spacingLg

        VirtualJoystick {
            Layout.preferredWidth: Theme.joystickSize
            Layout.preferredHeight: Theme.joystickSize
            Layout.alignment: Qt.AlignVCenter
            enabled: Backend.teleop.enabled
        }

        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: Theme.spacingSm

            RowLayout {
                Layout.fillWidth: true

                Text {
                    text: qsTr("Hız ölçeği")
                    color: Theme.textDim
                    font.pixelSize: Theme.fontSm
                }

                Item {
                    Layout.fillWidth: true
                }

                Text {
                    text: String(Math.round(Backend.teleop.speedScale * 100)).padStart(3, " ") + " %"
                    color: Theme.text
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontMd
                }
            }

            Slider {
                Layout.fillWidth: true
                from: Backend.teleop.scaleMin
                to: Backend.teleop.scaleMax
                stepSize: Backend.teleop.scaleStep
                snapMode: Slider.SnapAlways
                value: Backend.teleop.speedScale
                focusPolicy: Qt.NoFocus
                onMoved: Backend.teleop.speedScale = value
            }

            GridLayout {
                columns: 3
                columnSpacing: Theme.spacingLg
                rowSpacing: Theme.spacingXs

                Item {
                    Layout.preferredWidth: 1
                }

                Text {
                    text: qsTr("komut")
                    color: Theme.textDim
                    font.pixelSize: Theme.fontXs
                }

                Text {
                    text: qsTr("ölçülen")
                    color: Theme.textDim
                    font.pixelSize: Theme.fontXs
                }

                Text {
                    text: qsTr("v [m/s]")
                    color: Theme.textDim
                    font.pixelSize: Theme.fontSm
                }

                Text {
                    text: Theme.signedFixed(Backend.teleop.linearCommand, 3)
                    color: Theme.text
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontMd
                }

                Text {
                    text: Backend.telemetry.hasOdom ? Theme.signedFixed(Backend.telemetry.linear, 3) : "—"
                    color: Theme.measured
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontMd
                }

                Text {
                    text: qsTr("ω [rad/s]")
                    color: Theme.textDim
                    font.pixelSize: Theme.fontSm
                }

                Text {
                    text: Theme.signedFixed(Backend.teleop.angularCommand, 3)
                    color: Theme.text
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontMd
                }

                Text {
                    text: Backend.telemetry.hasOdom ? Theme.signedFixed(Backend.telemetry.angular, 3) : "—"
                    color: Theme.measured
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontMd
                }
            }

            Item {
                Layout.fillHeight: true
            }

            Text {
                Layout.fillWidth: true
                text: Backend.teleop.enabled ? qsTr("W/S ileri-geri · A/D dönüş · bırakınca durur") : qsTr("E-STOP etkin — sürüş devre dışı")
                color: Backend.teleop.enabled ? Theme.textFaint : Theme.danger
                font.pixelSize: Theme.fontXs
                elide: Text.ElideRight
            }
        }
    }
}
