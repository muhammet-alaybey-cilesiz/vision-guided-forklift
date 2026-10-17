pragma Singleton

import QtQuick

// Tüm görsel sabitler: renk, font ve boşluk değerleri yalnızca burada.
// Koyu, endüstriyel operatör paneli teması. Durum renkleri tüm arayüzde
// tutarlıdır: yeşil normal, sarı uyarı, kırmızı hata / e-stop, mavi bilgi.
QtObject {
    // --- Yüzeyler ---
    readonly property color background: "#0f1215"
    readonly property color panel: "#171b20"
    readonly property color panelHeader: "#1f252c"
    readonly property color raised: "#262d35"
    readonly property color raisedHover: "#2f3842"
    readonly property color pressed: "#3a4550"
    readonly property color border: "#323a44"
    readonly property color borderStrong: "#4a5562"
    readonly property color videoBackground: "#000000"

    // --- Metin ---
    readonly property color text: "#e4e8ec"
    readonly property color textDim: "#98a2ad"
    readonly property color textFaint: "#5f6a75"
    readonly property color textOnDanger: "#ffffff"

    // --- Durum renkleri ---
    readonly property color ok: "#2ecc71"
    readonly property color warn: "#f1c40f"
    readonly property color danger: "#e74c3c"
    readonly property color info: "#3498db"
    readonly property color idle: "#5d6670"
    readonly property color dangerDark: "#8e1f15"

    // --- Grafik ve kaplama renkleri ---
    readonly property color accent: "#3498db"
    readonly property color trail: "#3498db"
    readonly property color robot: "#f39c12"
    readonly property color target: "#3498db"
    readonly property color measured: "#2ecc71"
    readonly property color toleranceBand: "#332ecc71"
    readonly property color detection: "#ff3b6b"
    readonly property color crosshair: "#ccffffff"
    readonly property color overlayBackground: "#99000000"
    readonly property color grid: "#262d35"
    readonly property color scrim: "#cc0b0e11"

    // --- Fontlar: sayılar sabit genişlikli (değişirken titremez) ---
    readonly property string monoFamily: "monospace"
    readonly property int fontXs: 10
    readonly property int fontSm: 11
    readonly property int fontMd: 13
    readonly property int fontLg: 16
    readonly property int fontXl: 20
    readonly property int fontXxl: 26

    // --- Boşluk ve ölçüler [px] ---
    readonly property int spacingXs: 2
    readonly property int spacingSm: 4
    readonly property int spacing: 8
    readonly property int spacingLg: 12
    readonly property int margin: 10
    readonly property int radius: 4
    readonly property int panelHeaderHeight: 28
    readonly property int topBarHeight: 56
    readonly property int estopFrameWidth: 6
    readonly property int ledSize: 12
    readonly property int buttonHeight: 30
    readonly property int tabHeight: 34
    readonly property int estopButtonWidth: 190
    readonly property int estopButtonHeight: 44
    readonly property int joystickSize: 150
    readonly property int joystickKnob: 54
    readonly property int gaugeWidth: 64
    readonly property int splitHandle: 4

    // Sağlık durumu → renk (TopicHealthModel.State: 0 bekleniyor, 1 normal,
    // 2 uyarı, 3 hata). Yalnız görsel eşleme; durum hesabı C++ core'da.
    function stateColor(state: int): color {
        switch (state) {
        case 1:
            return ok;
        case 2:
            return warn;
        case 3:
            return danger;
        default:
            return idle;
        }
    }

    // Sağlık durumu → kısa ad (yalnız görsel).
    function statusName(state: int): string {
        switch (state) {
        case 1:
            return qsTr("normal");
        case 2:
            return qsTr("uyarı");
        case 3:
            return qsTr("hata");
        default:
            return qsTr("bekleniyor");
        }
    }

    // İşaretli sabit genişlikli sayı: "+0.123" / "−0.123"; değer değişirken
    // metin genişliği sabit kalır (titremez). -0.000 "+0.000" gösterilir.
    function signedFixed(value: real, decimals: int): string {
        const text = Math.abs(value).toFixed(decimals);
        const zero = Number(text) === 0;
        return (value < 0 && !zero ? "−" : "+") + text;
    }

    // Sabit ondalıklı sayı; "-0.0" yerine "0.0" (ör. dinlenen çatal eklemi
    // −1e-14 m okur, D-10).
    function fixed(value: real, decimals: int): string {
        const text = value.toFixed(decimals);
        return Number(text) === 0 ? (0).toFixed(decimals) : text;
    }

    // /rosout seviyesi → renk (rcl_interfaces/Log: 10 debug … 50 fatal).
    function levelColor(level: int): color {
        if (level >= 40)
            return danger;
        if (level >= 30)
            return warn;
        if (level >= 20)
            return text;
        return textFaint;
    }
}
