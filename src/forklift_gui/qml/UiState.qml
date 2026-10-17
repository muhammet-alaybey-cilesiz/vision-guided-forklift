pragma Singleton

import QtQuick

// Yalnız görünüm tercihleri (iş mantığı yok): kamera düzeni, sekme modunda
// seçili kamera ve artı işareti. Delegate'ler dış id'lere erişmesin diye
// singleton'da (Qt 6.4'te ComponentBehavior yok).
QtObject {
    // 0 yan yana, 1 üst üste, 2 sekme
    property int cameraLayout: 0
    property int currentCamera: 0
    property bool crosshair: true
}
