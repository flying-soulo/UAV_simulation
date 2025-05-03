const socket = io();

socket.on("telemetry", function(data) {
    for (const key in data) {
        const el = document.getElementById(key);
        if (el) el.textContent = data[key];
    }
});

function startSim() {
    fetch("/start").then(res => console.log("Sim started"));
}
