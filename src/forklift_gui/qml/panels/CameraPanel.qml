import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import Forklift.Core
import Forklift.Gui

// Kameralar: düzen seçimi (yan yana / üst üste / sekme), artı işareti ve
// her kamera için CameraView (spesifikasyon §4.4).
PanelFrame {
    id: panel

    title: qsTr("Kameralar")
    headerExtras: [
        Row {
            spacing: Theme.spacingXs

            Repeater {
                model: [qsTr("Yan yana"), qsTr("Üst üste"), qsTr("Sekme")]

                delegate: Button {
                    required property int index
                    required property string modelData

                    implicitHeight: Theme.panelHeaderHeight - 6
                    text: modelData
                    font.pixelSize: Theme.fontXs
                    flat: UiState.cameraLayout !== index
                    focusPolicy: Qt.NoFocus
                    onClicked: UiState.cameraLayout = index
                }
            }

            Button {
                implicitHeight: Theme.panelHeaderHeight - 6
                text: UiState.crosshair ? qsTr("Artı ✓") : qsTr("Artı")
                font.pixelSize: Theme.fontXs
                flat: !UiState.crosshair
                focusPolicy: Qt.NoFocus
                onClicked: UiState.crosshair = !UiState.crosshair
            }
        }
    ]

    ColumnLayout {
        anchors.fill: parent
        spacing: Theme.spacingSm

        TabBar {
            Layout.fillWidth: true
            visible: UiState.cameraLayout === 2

            background: Rectangle {
                color: Theme.panelHeader
            }
            currentIndex: UiState.currentCamera
            focusPolicy: Qt.NoFocus
            onCurrentIndexChanged: UiState.currentCamera = currentIndex

            Repeater {
                model: Backend.cameras

                delegate: ThemedTabButton {
                    required property CameraStream modelData

                    text: modelData.label
                }
            }
        }

        GridLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            columns: UiState.cameraLayout === 0 ? Math.max(1, Backend.cameras.length) : 1
            columnSpacing: Theme.spacingSm
            rowSpacing: Theme.spacingSm

            Repeater {
                model: Backend.cameras

                delegate: CameraView {
                    required property CameraStream modelData
                    required property int index

                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 1
                    Layout.preferredHeight: 1
                    stream: modelData
                    visible: UiState.cameraLayout !== 2 || index === UiState.currentCamera
                }
            }
        }
    }
}
