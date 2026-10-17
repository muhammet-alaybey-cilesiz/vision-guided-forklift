import QtQuick
import Forklift.Core
import Forklift.Gui

// Büyük kırmızı acil durdurma butonu. Serbestken tek basış etkinleştirir
// (Space de aynı). Etkinken serbest bırakmak için release_hold süre basılı
// tutulur; ilerleme çubuğu dolar. Mantık C++ SafetyController'da (D-31).
Rectangle {
    id: button

    readonly property bool engaged: Backend.safety.engaged

    implicitWidth: Theme.estopButtonWidth
    implicitHeight: Theme.estopButtonHeight
    radius: Theme.radius
    color: button.engaged ? Theme.dangerDark : (area.pressed ? Theme.dangerDark : Theme.danger)
    border.width: 2
    border.color: button.engaged ? Theme.danger : Qt.lighter(Theme.danger, 1.3)

    // Bırakma ilerlemesi
    Rectangle {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.margins: 2
        width: (parent.width - 4) * Backend.safety.releaseProgress
        radius: Theme.radius
        color: Theme.ok
        opacity: 0.55
        visible: Backend.safety.releasing
    }

    Column {
        anchors.centerIn: parent
        spacing: 0

        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            text: button.engaged ? qsTr("E-STOP ETKİN") : qsTr("E-STOP  [Space]")
            color: Theme.textOnDanger
            font.pixelSize: Theme.fontLg
            font.bold: true
        }

        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            visible: button.engaged
            text: qsTr("Bırakmak için %1 s basılı tut").arg(Backend.safety.releaseHoldSeconds.toFixed(1))
            color: Theme.textOnDanger
            font.pixelSize: Theme.fontXs
        }
    }

    MouseArea {
        id: area

        anchors.fill: parent
        cursorShape: Qt.PointingHandCursor
        onPressed: {
            if (button.engaged)
                Backend.safety.beginRelease();
            else
                Backend.safety.engage();
        }
        onReleased: Backend.safety.cancelRelease()
        onCanceled: Backend.safety.cancelRelease()
    }
}
