import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import Forklift.Core
import Forklift.Gui

// /rosout ve GUI olayları: seviye renkli, düğüm adı, mesaj; seviye filtresi
// ve metin araması C++ LogFilterModel'de (spesifikasyon §4.8).
PanelFrame {
    id: panel

    title: qsTr("Log")

    ColumnLayout {
        anchors.fill: parent
        spacing: Theme.spacingSm

        RowLayout {
            Layout.fillWidth: true
            spacing: Theme.spacingSm

            ComboBox {
                id: level

                // rcl_interfaces/Log seviyeleri: 10 debug … 40 error
                readonly property var levels: [0, 10, 20, 30, 40]

                Layout.preferredWidth: 120
                model: [qsTr("Tümü"), "DEBUG+", "INFO+", "WARN+", "ERROR+"]
                focusPolicy: Qt.NoFocus
                onActivated: function (index) {
                    Backend.logView.minLevel = level.levels[index];
                }
            }

            TextField {
                id: search

                Layout.fillWidth: true
                placeholderText: qsTr("Ara: düğüm ya da mesaj")
                onTextChanged: Backend.logView.search = search.text
                Keys.onEscapePressed: search.focus = false
            }

            CheckBox {
                id: follow

                text: qsTr("Takip")
                checked: true
                focusPolicy: Qt.NoFocus
            }

            Text {
                text: list.count + " / " + Backend.log.count
                color: Theme.textDim
                font.family: Theme.monoFamily
                font.pixelSize: Theme.fontSm
            }

            Button {
                text: qsTr("Temizle")
                focusPolicy: Qt.NoFocus
                onClicked: Backend.log.clear()
            }
        }

        ListView {
            id: list

            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            model: Backend.logView
            boundsBehavior: Flickable.StopAtBounds
            onCountChanged: {
                if (follow.checked)
                    list.positionViewAtEnd();
            }

            ScrollBar.vertical: ScrollBar {}

            delegate: RowLayout {
                id: entry

                required property string time
                required property int level
                required property string levelName
                required property string node
                required property string message
                required property bool local

                width: ListView.view.width
                spacing: Theme.spacing

                Text {
                    text: entry.time
                    color: Theme.textFaint
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontSm
                }

                Text {
                    Layout.preferredWidth: 48
                    text: entry.levelName
                    color: Theme.levelColor(entry.level)
                    font.family: Theme.monoFamily
                    font.pixelSize: Theme.fontSm
                    font.bold: entry.level >= 30
                }

                Text {
                    Layout.preferredWidth: 130
                    text: entry.node
                    color: entry.local ? Theme.accent : Theme.textDim
                    font.pixelSize: Theme.fontSm
                    elide: Text.ElideRight
                }

                Text {
                    Layout.fillWidth: true
                    text: entry.message
                    color: Theme.levelColor(entry.level)
                    font.pixelSize: Theme.fontSm
                    elide: Text.ElideRight
                }
            }
        }
    }
}
