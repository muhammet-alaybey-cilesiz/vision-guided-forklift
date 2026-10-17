import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import Forklift.Gui

// Alt sekmeli panel: Sağlık / Log / Veri seti.
Rectangle {
    color: Theme.panel
    border.color: Theme.border
    border.width: 1
    radius: Theme.radius

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 1
        spacing: 0

        TabBar {
            id: tabs

            Layout.fillWidth: true
            focusPolicy: Qt.NoFocus

            background: Rectangle {
                color: Theme.panelHeader
            }

            ThemedTabButton {
                text: qsTr("Sağlık")
            }
            ThemedTabButton {
                text: qsTr("Log")
            }
            ThemedTabButton {
                text: qsTr("Veri seti")
            }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: tabs.currentIndex

            HealthPanel {}
            LogPanel {}
            DatasetPanel {}
        }
    }
}
