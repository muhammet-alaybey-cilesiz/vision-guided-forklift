import QtQuick
import QtQuick.Controls.Basic
import Forklift.Gui

// Koyu temaya uygun sekme düğmesi: seçili sekme panel renginde ve altı vurgulu.
TabButton {
    id: tab

    focusPolicy: Qt.NoFocus
    font.pixelSize: Theme.fontMd

    contentItem: Text {
        text: tab.text
        font: tab.font
        color: tab.checked ? Theme.text : Theme.textDim
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }

    background: Rectangle {
        implicitHeight: Theme.tabHeight
        color: tab.checked ? Theme.panel : (tab.hovered ? Theme.raised : Theme.panelHeader)

        Rectangle {
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            height: 2
            color: Theme.accent
            visible: tab.checked
        }
    }
}
