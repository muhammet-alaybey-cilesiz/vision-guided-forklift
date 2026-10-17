import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import Forklift.Core
import Forklift.Gui

// Sistem sağlığı: her topic için LED, ölçülen ve beklenen frekans, son mesaj
// yaşı (spesifikasyon §4.7). Durum C++ TopicHealthModel'de hesaplanır (D-35).
PanelFrame {
    id: panel

    title: qsTr("Sistem sağlığı")

    ColumnLayout {
        anchors.fill: parent
        spacing: Theme.spacingXs

        RowLayout {
            Layout.fillWidth: true
            spacing: Theme.spacing

            Item {
                Layout.preferredWidth: Theme.ledSize
            }

            Text {
                Layout.fillWidth: true
                text: qsTr("topic")
                color: Theme.textDim
                font.pixelSize: Theme.fontXs
            }

            Text {
                Layout.preferredWidth: 70
                horizontalAlignment: Text.AlignRight
                text: qsTr("ölçülen Hz")
                color: Theme.textDim
                font.pixelSize: Theme.fontXs
            }

            Text {
                Layout.preferredWidth: 70
                horizontalAlignment: Text.AlignRight
                text: qsTr("beklenen")
                color: Theme.textDim
                font.pixelSize: Theme.fontXs
            }

            Text {
                Layout.preferredWidth: 70
                horizontalAlignment: Text.AlignRight
                text: qsTr("son mesaj")
                color: Theme.textDim
                font.pixelSize: Theme.fontXs
            }

            Text {
                Layout.preferredWidth: 110
                text: qsTr("durum")
                color: Theme.textDim
                font.pixelSize: Theme.fontXs
            }
        }

        ListView {
            id: list

            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            model: Backend.health
            boundsBehavior: Flickable.StopAtBounds

            ScrollBar.vertical: ScrollBar {}

            delegate: RowLayout {
                id: row

                required property string topic
                required property int status
                required property real rate
                required property real age
                required property real expectedRate
                required property bool essential

                width: ListView.view.width
                spacing: Theme.spacing

                Rectangle {
                    Layout.preferredWidth: Theme.ledSize
                    Layout.preferredHeight: Theme.ledSize
                    radius: width / 2
                    color: Theme.stateColor(row.status)
                }

                Text {
                    Layout.fillWidth: true
                    text: row.topic
                    color: row.essential ? Theme.text : Theme.textDim
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontSm
                    elide: Text.ElideMiddle
                }

                Text {
                    Layout.preferredWidth: 70
                    horizontalAlignment: Text.AlignRight
                    text: row.age < 0 ? "—" : row.rate.toFixed(1)
                    color: Theme.text
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontSm
                }

                Text {
                    Layout.preferredWidth: 70
                    horizontalAlignment: Text.AlignRight
                    text: row.expectedRate > 0 ? row.expectedRate.toFixed(0) : qsTr("olay")
                    color: Theme.textDim
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontSm
                }

                Text {
                    Layout.preferredWidth: 70
                    horizontalAlignment: Text.AlignRight
                    text: row.age < 0 ? "—" : (row.age < 100 ? row.age.toFixed(2) + " s" : "> 100 s")
                    color: Theme.text
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontSm
                }

                Text {
                    Layout.preferredWidth: 110
                    text: Theme.statusName(row.status) + (row.essential ? "" : qsTr(" (ops.)"))
                    color: Theme.stateColor(row.status)
                    font.pixelSize: Theme.fontSm
                }
            }
        }
    }
}
