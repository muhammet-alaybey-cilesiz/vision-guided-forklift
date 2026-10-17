import QtQuick
import Forklift.Gui

// Odometri izinin 2B kuş bakışı çizimi: son N nokta (C++ TelemetryModel),
// otomatik ölçek (eşit eksen), robotun yön oku ve ölçek çubuğu.
Canvas {
    id: canvas

    property var points: []
    property real minX: 0
    property real maxX: 0
    property real minY: 0
    property real maxY: 0
    property real robotX: 0
    property real robotY: 0
    property real robotYaw: 0
    property bool hasPose: false
    // En küçük görünür alan [m]; robot dururken ölçek patlamasın.
    property real minSpan: 1.0

    onPointsChanged: canvas.requestPaint()
    onRobotXChanged: canvas.requestPaint()
    onRobotYChanged: canvas.requestPaint()
    onRobotYawChanged: canvas.requestPaint()
    onWidthChanged: canvas.requestPaint()
    onHeightChanged: canvas.requestPaint()

    onPaint: {
        const ctx = canvas.getContext("2d");
        ctx.reset();
        ctx.fillStyle = Theme.videoBackground;
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        if (!canvas.hasPose)
            return;
        // Görünür alan: iz sınırları + robot, en az minSpan; eşit eksen.
        let minX = Math.min(canvas.minX, canvas.robotX);
        let maxX = Math.max(canvas.maxX, canvas.robotX);
        let minY = Math.min(canvas.minY, canvas.robotY);
        let maxY = Math.max(canvas.maxY, canvas.robotY);
        if (canvas.points.length === 0) {
            minX = maxX = canvas.robotX;
            minY = maxY = canvas.robotY;
        }
        const cxw = (minX + maxX) / 2;
        const cyw = (minY + maxY) / 2;
        const span = Math.max(canvas.minSpan, maxX - minX, maxY - minY) * 1.25;
        const margin = 12;
        const s = Math.min(canvas.width - 2 * margin, canvas.height - 2 * margin) / span;
        const px = function (x) { return canvas.width / 2 + (x - cxw) * s; };
        const py = function (y) { return canvas.height / 2 - (y - cyw) * s; };

        // Izgara: 1 m (ya da ölçeğe göre) aralık
        let step = 0.25;
        while (step * s < 40)
            step *= 2;
        ctx.strokeStyle = Theme.grid;
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let gx = Math.floor((cxw - span) / step) * step; gx <= cxw + span; gx += step) {
            ctx.moveTo(px(gx), 0);
            ctx.lineTo(px(gx), canvas.height);
        }
        for (let gy = Math.floor((cyw - span) / step) * step; gy <= cyw + span; gy += step) {
            ctx.moveTo(0, py(gy));
            ctx.lineTo(canvas.width, py(gy));
        }
        ctx.stroke();

        // İz
        if (canvas.points.length > 1) {
            ctx.strokeStyle = Theme.trail;
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(px(canvas.points[0].x), py(canvas.points[0].y));
            for (let i = 1; i < canvas.points.length; ++i)
                ctx.lineTo(px(canvas.points[i].x), py(canvas.points[i].y));
            ctx.stroke();
        }

        // Robot ve yön oku
        const rx = px(canvas.robotX);
        const ry = py(canvas.robotY);
        const len = 16;
        const c = Math.cos(canvas.robotYaw);
        const sn = Math.sin(canvas.robotYaw);
        ctx.fillStyle = Theme.robot;
        ctx.beginPath();
        ctx.moveTo(rx + c * len, ry - sn * len);
        ctx.lineTo(rx - c * len * 0.6 - sn * len * 0.5, ry + sn * len * 0.6 - c * len * 0.5);
        ctx.lineTo(rx - c * len * 0.6 + sn * len * 0.5, ry + sn * len * 0.6 + c * len * 0.5);
        ctx.closePath();
        ctx.fill();

        // Ölçek çubuğu
        ctx.strokeStyle = Theme.textDim;
        ctx.fillStyle = Theme.textDim;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(margin, canvas.height - margin);
        ctx.lineTo(margin + step * s, canvas.height - margin);
        ctx.stroke();
        ctx.font = Theme.fontXs + "px " + Theme.monoFamily;
        ctx.fillText(step + " m", margin, canvas.height - margin - 4);
    }
}
