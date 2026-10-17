import QtQuick
import QtQuick.Layouts
import Forklift.Core
import Forklift.Gui

// Telemetri: poz, hız, mesafe, çatal, tekerlekler, sim zamanı / RTF ve
// odometri izi (spesifikasyon §4.6). Değerler C++'ta ≤ 30 Hz güncellenir.
PanelFrame {
    id: panel

    readonly property bool odom: Backend.telemetry.hasOdom

    title: qsTr("Telemetri")

    ColumnLayout {
        anchors.fill: parent
        spacing: Theme.spacing

        GridLayout {
            Layout.fillWidth: true
            columns: 3
            columnSpacing: Theme.spacingLg
            rowSpacing: Theme.spacingSm

            ValueTile {
                label: "x"
                value: panel.odom ? Theme.signedFixed(Backend.telemetry.x, 3) : "—"
                unit: "m"
            }

            ValueTile {
                label: "y"
                value: panel.odom ? Theme.signedFixed(Backend.telemetry.y, 3) : "—"
                unit: "m"
            }

            ValueTile {
                label: qsTr("yaw")
                value: panel.odom ? Theme.signedFixed(Backend.telemetry.yawDegrees, 1).padStart(6, " ") : "—"
                unit: "°"
            }

            ValueTile {
                label: "v"
                value: panel.odom ? Theme.signedFixed(Backend.telemetry.linear, 3) : "—"
                unit: "m/s"
                valueColor: Theme.measured
            }

            ValueTile {
                label: "ω"
                value: panel.odom ? Theme.signedFixed(Backend.telemetry.angular, 3) : "—"
                unit: "rad/s"
                valueColor: Theme.measured
            }

            ValueTile {
                label: qsTr("mesafe")
                value: panel.odom ? Backend.telemetry.distance.toFixed(2) : "—"
                unit: "m"
            }

            ValueTile {
                label: qsTr("çatal")
                value: Backend.telemetry.hasFork ? Theme.fixed(Backend.telemetry.forkHeight * 1000, 1) : "—"
                unit: "mm"
            }

            ValueTile {
                label: qsTr("teker sol / sağ")
                value: Backend.telemetry.hasWheels ? Theme.signedFixed(Backend.telemetry.leftWheel, 1) + " " + Theme.signedFixed(Backend.telemetry.rightWheel, 1) : "—"
                unit: "rad/s"
            }

            ValueTile {
                label: qsTr("sim / RTF")
                value: (Backend.telemetry.hasClock ? Backend.telemetry.simTime.toFixed(1) : "—") + " / " + (Backend.telemetry.rtfValid ? Backend.telemetry.rtf.toFixed(2) : "—")
                unit: "s"
            }
        }

        OdomTrail {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 80
            points: Backend.telemetry.trailPoints
            minX: Backend.telemetry.trailMinX
            maxX: Backend.telemetry.trailMaxX
            minY: Backend.telemetry.trailMinY
            maxY: Backend.telemetry.trailMaxY
            robotX: Backend.telemetry.x
            robotY: Backend.telemetry.y
            robotYaw: Backend.telemetry.yaw
            hasPose: panel.odom
        }
    }
}
