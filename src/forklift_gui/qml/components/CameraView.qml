import QtQuick
import QtMultimedia
import Forklift.Core
import Forklift.Gui

// Tek kamera görüntüsü: VideoOutput (en-boy korunur), merkez artı işareti,
// tespit kaplaması, bilgi şeridi (FPS, çözünürlük, son kare yaşı) ve STALE
// uyarısı. Kare dönüşümü ROS thread'inde yapılmıştır; burada yalnız gösterim.
Rectangle {
    id: view

    property CameraStream stream: null

    color: Theme.videoBackground
    clip: true

    VideoOutput {
        id: video

        anchors.fill: parent
        fillMode: VideoOutput.PreserveAspectFit
    }

    // Hazır kareler bu sink'e verilir (CameraStream::onFrame).
    Binding {
        target: view.stream
        property: "videoSink"
        value: video.videoSink
        when: view.stream !== null
    }

    // Görüntünün ekrandaki dikdörtgeni: kaplamalar bunun içinde çizilir.
    Item {
        id: content

        x: video.contentRect.x
        y: video.contentRect.y
        width: video.contentRect.width
        height: video.contentRect.height

        Item {
            anchors.fill: parent
            visible: UiState.crosshair && view.stream !== null && view.stream.hasFrame

            Rectangle {
                anchors.centerIn: parent
                width: Math.min(parent.width, parent.height) * 0.12
                height: 2
                color: Theme.crosshair
            }

            Rectangle {
                anchors.centerIn: parent
                width: 2
                height: Math.min(parent.width, parent.height) * 0.12
                color: Theme.crosshair
            }
        }

        DetectionOverlay {
            anchors.fill: parent
            boxes: view.stream !== null ? view.stream.detections : []
            visible: view.stream !== null && view.stream.showsDetections && view.stream.detectionsVisible
        }
    }

    // Bilgi şeridi
    Rectangle {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: Theme.spacingSm
        width: info.implicitWidth + 2 * Theme.spacing
        height: info.implicitHeight + Theme.spacingSm
        radius: Theme.radius
        color: Theme.overlayBackground
        visible: view.stream !== null

        Text {
            id: info

            anchors.centerIn: parent
            text: view.stream === null ? "" : view.stream.label + "   " + view.stream.fps.toFixed(1).padStart(4, " ") + " FPS   " + (view.stream.frameWidth > 0 ? view.stream.frameWidth + "×" + view.stream.frameHeight : (view.stream.infoWidth > 0 ? view.stream.infoWidth + "×" + view.stream.infoHeight : "—")) + "   " + (view.stream.frameAge >= 0 ? view.stream.frameAge.toFixed(2) + " s" : "—")
            color: Theme.text
            font.family: Theme.monoFamily
            font.pixelSize: Theme.fontSm
        }
    }

    // STALE: son kare stale_after'dan eski (ya da hiç kare yok)
    Rectangle {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: Theme.spacingSm
        width: staleText.implicitWidth + 2 * Theme.spacing
        height: staleText.implicitHeight + Theme.spacingSm
        radius: Theme.radius
        color: view.stream !== null && view.stream.hasFrame ? Theme.danger : Theme.idle
        visible: view.stream !== null && view.stream.stale

        Text {
            id: staleText

            anchors.centerIn: parent
            text: view.stream !== null && view.stream.hasFrame ? qsTr("STALE") : qsTr("görüntü bekleniyor")
            color: Theme.textOnDanger
            font.pixelSize: Theme.fontSm
            font.bold: true
        }
    }
}
