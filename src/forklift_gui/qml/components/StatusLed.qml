import QtQuick
import Forklift.Gui

// Durum LED'i ve etiketi. status: TopicHealthModel.State (0 bekleniyor,
// 1 normal, 2 uyarı, 3 hata); ledColor ile renk ezilebilir (ör. MOCK mavi).
Row {
    id: led

    property int status: 0
    property string label
    property color ledColor: Theme.stateColor(led.status)

    spacing: Theme.spacingSm

    Rectangle {
        anchors.verticalCenter: parent.verticalCenter
        width: Theme.ledSize
        height: Theme.ledSize
        radius: width / 2
        color: led.ledColor
        border.width: 1
        border.color: Qt.darker(led.ledColor, 1.8)
    }

    Text {
        anchors.verticalCenter: parent.verticalCenter
        text: led.label
        color: Theme.text
        font.pixelSize: Theme.fontSm
        font.bold: true
    }
}
