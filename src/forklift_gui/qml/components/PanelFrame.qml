import QtQuick
import Forklift.Gui

// Panel çerçevesi: başlık şeridi (sağda isteğe bağlı öğeler) ve içerik alanı.
Rectangle {
    id: frame

    property string title
    default property alias content: body.data
    property alias headerExtras: extras.data

    color: Theme.panel
    border.color: Theme.border
    border.width: 1
    radius: Theme.radius
    clip: true

    Rectangle {
        id: header

        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 1
        height: Theme.panelHeaderHeight
        color: Theme.panelHeader
        radius: Theme.radius

        Text {
            anchors.left: parent.left
            anchors.leftMargin: Theme.margin
            anchors.verticalCenter: parent.verticalCenter
            text: frame.title
            color: Theme.textDim
            font.pixelSize: Theme.fontSm
            font.bold: true
            font.capitalization: Font.AllUppercase
        }

        Row {
            id: extras

            anchors.right: parent.right
            anchors.rightMargin: Theme.spacing
            anchors.verticalCenter: parent.verticalCenter
            spacing: Theme.spacingSm
        }
    }

    Item {
        id: body

        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: header.bottom
        anchors.bottom: parent.bottom
        anchors.margins: Theme.spacing
    }
}
