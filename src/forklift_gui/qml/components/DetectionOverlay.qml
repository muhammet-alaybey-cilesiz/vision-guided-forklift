import QtQuick
import Forklift.Gui

// Tespit kutuları (/cv/detections). CameraStream kutuları görüntü boyutuna
// normalize eder (0–1); burada yalnız içerik dikdörtgeninin boyutuyla
// çarpılır (D-34). Etiket: sınıf ve skor.
Item {
    id: overlay

    property var boxes: []

    Repeater {
        model: overlay.boxes

        delegate: Rectangle {
            id: box

            required property var modelData

            x: box.modelData.x * parent.width
            y: box.modelData.y * parent.height
            width: box.modelData.w * parent.width
            height: box.modelData.h * parent.height
            color: "transparent"
            border.width: 2
            border.color: Theme.detection

            Rectangle {
                anchors.left: parent.left
                anchors.bottom: parent.top
                width: label.implicitWidth + Theme.spacing
                height: label.implicitHeight + 2
                color: Theme.detection

                Text {
                    id: label

                    anchors.centerIn: parent
                    text: box.modelData.label + " " + Number(box.modelData.score).toFixed(2)
                    color: Theme.textOnDanger
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontXs
                    font.bold: true
                }
            }
        }
    }
}
