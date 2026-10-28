'use strict';
const $ = id => document.getElementById(id);
const storageKey = 'cubesat-live-layout-v1';
let catalog = [], layout = [], paused = false, socket, sessionToken;
let lastStatus = '', lastChange = Date.now(), reconnectTimer;
const cards = new Map();

function persist() {
    try { localStorage.setItem(storageKey, JSON.stringify(layout)); } catch (_) {}
}
function validate(value) {
    if (!Array.isArray(value)) throw Error('Layout must be an array');
    const seen = new Set();
    return value.filter(x => x && catalog.some(c => c.id === x.id) && !seen.has(x.id) && seen.add(x.id))
        .map(x => ({id: x.id, width: Number.isFinite(x.width) ? Math.max(300, Math.min(1800, x.width)) : null,
                    height: Number.isFinite(x.height) ? Math.max(230, Math.min(1200, x.height)) : 350}));
}
function send(message) {
    if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}
function subscribe() {
    send({type: 'subscribe', ids: layout.map(x => x.id), paused, interval: Number($('interval').value) / 1000});
}
function library() {
    const query = $('search').value.toLowerCase();
    $('library').replaceChildren();
    let group = '';
    for (const item of catalog.filter(x => (x.group + ' ' + x.title).toLowerCase().includes(query))) {
        if (group !== item.group) {
            group = item.group;
            const heading = document.createElement('h2'); heading.textContent = group;
            $('library').append(heading);
        }
        const button = document.createElement('button');
        button.className = 'catalog-item'; button.textContent = '+ ' + item.title;
        button.disabled = layout.some(x => x.id === item.id); button.draggable = true;
        button.ondragstart = event => event.dataTransfer.setData('text/plain', item.id);
        button.onclick = () => add(item.id);
        $('library').append(button);
    }
}
function add(id, before) {
    if (!catalog.some(x => x.id === id)) return;
    const item = layout.find(x => x.id === id) || {id, height: 350};
    layout = layout.filter(x => x.id !== id);
    const index = layout.findIndex(x => x.id === before);
    layout.splice(index < 0 ? layout.length : index, 0, item);
    persist(); draw(); subscribe();
}
function move(id, delta) {
    const i = layout.findIndex(x => x.id === id), j = i + delta;
    if (j < 0 || j >= layout.length) return;
    [layout[i], layout[j]] = [layout[j], layout[i]];
    persist(); draw();
}
const resizeFrames = new Map();
const observer = new ResizeObserver(entries => {
    for (const entry of entries) {
        const card = entry.target, item = layout.find(x => x.id === card.dataset.id);
        if (!item) continue;
        item.width = Math.round(card.offsetWidth); item.height = Math.round(card.offsetHeight);
        cancelAnimationFrame(resizeFrames.get(item.id));
        resizeFrames.set(item.id, requestAnimationFrame(() => {
            const canvas = card.querySelector('.plot-canvas');
            if (canvas.data && canvas.isConnected) Plotly.Plots.resize(canvas);
        }));
    }
    persist();
});
function createCard(item) {
    const card = document.createElement('article'); card.dataset.id = item.id;
    if (item.width) card.style.width = item.width + 'px';
    card.style.height = item.height + 'px';
    const head = document.createElement('div'); head.className = 'card-head'; head.draggable = true;
    head.ondragstart = event => event.dataTransfer.setData('text/plain', item.id);
    const title = document.createElement('strong'); title.textContent = catalog.find(x => x.id === item.id).title;
    head.append(title);
    for (const [label, description, action] of [
        ['←', 'Move earlier', () => move(item.id, -1)], ['→', 'Move later', () => move(item.id, 1)],
        ['×', 'Remove', () => { layout = layout.filter(x => x.id !== item.id); persist(); draw(); subscribe(); }]
    ]) {
        const button = document.createElement('button'); button.textContent = label;
        button.setAttribute('aria-label', description + ' ' + title.textContent); button.onclick = action;
        head.append(button);
    }
    const plot = document.createElement('div'); plot.className = 'plot';
    const canvas = document.createElement('div'); canvas.className = 'plot-canvas';
    canvas.setAttribute('aria-label', title.textContent);
    const message = document.createElement('div'); message.className = 'error'; message.textContent = 'Waiting for live data…';
    plot.append(canvas, message); card.append(head, plot);
    card.ondragover = event => event.preventDefault();
    card.ondrop = event => {
        event.preventDefault(); event.stopPropagation();
        const id = event.dataTransfer.getData('text/plain'); if (id !== item.id) add(id, item.id);
    };
    observer.observe(card);
    return card;
}
function draw() {
    // Keep plot nodes alive on reorder so camera, zoom and legend state survive.
    for (const [id, card] of cards) {
        if (layout.some(x => x.id === id)) continue;
        observer.unobserve(card); cancelAnimationFrame(resizeFrames.get(id)); resizeFrames.delete(id);
        Plotly.purge(card.querySelector('.plot-canvas')); card.remove(); cards.delete(id);
    }
    $('empty')?.remove();
    for (const item of layout) {
        if (!cards.has(item.id)) cards.set(item.id, createCard(item));
        $('grid').append(cards.get(item.id));
    }
    if (!layout.length) {
        const empty = document.createElement('div'); empty.id = 'empty';
        empty.textContent = 'Drop graphs here to build your live dashboard'; $('grid').append(empty);
    }
    library();
}
function showSession(data) {
    sessionToken = data.token;
    $('start').hidden = data.state === 'viewer-only'; $('start').disabled = data.state !== 'waiting';
    const labels = {waiting: 'Ready — simulation has not started', starting: 'Starting simulation…',
        running: 'Simulation running', completed: 'Simulation completed', failed: 'Simulation failed'};
    $('simulation-state').textContent = (labels[data.state] || '') + (data.error ? ': ' + data.error : '');
}
function styledLayout(panel, id) {
    const value = {...panel.layout, autosize: true, paper_bgcolor: '#121f32', plot_bgcolor: '#121f32',
        font: {color: '#cbd8eb', family: 'system-ui, sans-serif', size: 11},
        margin: {l: 65, r: panel.layout.yaxis2 ? 65 : 25, t: 25, b: 55},
        legend: {orientation: 'h', y: 1.15}, uirevision: id, datarevision: panel.version};
    for (const axis of ['xaxis', 'yaxis', 'yaxis2']) {
        if (value[axis]) value[axis] = {gridcolor: '#293b51', zerolinecolor: '#64748b', ...value[axis]};
    }
    if (value.scene) {
        value.scene = {...value.scene, bgcolor: '#121f32'};
        for (const axis of ['xaxis', 'yaxis', 'zaxis']) {
            value.scene[axis] = {gridcolor: '#35465e', backgroundcolor: '#121f32', ...value.scene[axis]};
        }
    }
    return value;
}
async function update(payload) {
    showSession(payload.session);
    if (payload.status !== lastStatus) { lastStatus = payload.status; lastChange = Date.now(); }
    $('status').textContent = '● WebSocket connected · ' + payload.status +
        (Date.now() - lastChange > 10000 ? ' · no new samples' : '') + (paused ? ' · display paused' : '');
    $('estimator-status').textContent = 'Estimator: ' + payload.estimator_status;
    if (paused) return;
    for (const [id, panel] of Object.entries(payload.plots)) {
        const card = cards.get(id); if (!card) continue;
        const message = card.querySelector('.error'), canvas = card.querySelector('.plot-canvas');
        if (panel.error) { message.textContent = panel.error; message.hidden = false; continue; }
        try {
            await Plotly.react(canvas, panel.data, styledLayout(panel, id),
                {responsive: true, displaylogo: false, scrollZoom: true});
            message.hidden = true;
        } catch (error) { message.textContent = 'Plot error: ' + error.message; message.hidden = false; }
    }
}
function connect() {
    clearTimeout(reconnectTimer);
    const connection = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
    socket = connection;
    connection.onopen = subscribe;
    connection.onmessage = async event => {
        try {
            const payload = JSON.parse(event.data);
            if (payload.type === 'error') { $('status').textContent = payload.error; return; }
            await update(payload);
        } catch (error) { $('status').textContent = 'Stream error: ' + error.message; }
        finally { if (connection.readyState === WebSocket.OPEN) connection.send(JSON.stringify({type: 'ack'})); }
    };
    connection.onclose = () => {
        $('status').textContent = 'Disconnected — reconnecting…'; $('start').disabled = true;
        reconnectTimer = setTimeout(connect, 1000);
    };
    connection.onerror = () => connection.close();
}
$('grid').ondragover = event => event.preventDefault();
$('grid').ondrop = event => { event.preventDefault(); add(event.dataTransfer.getData('text/plain')); };
$('search').oninput = library;
$('interval').onchange = subscribe;
$('pause').onclick = () => { paused = !paused; $('pause').textContent = paused ? 'Resume display' : 'Pause display'; subscribe(); };
$('start').onclick = () => {
    $('start').disabled = true; paused = false; $('pause').textContent = 'Pause display';
    send({type: 'start', token: sessionToken}); subscribe();
};
$('follow').onclick = () => {
    for (const card of cards.values()) {
        const canvas = card.querySelector('.plot-canvas');
        if (canvas.data) Plotly.relayout(canvas, canvas.layout.scene ? {'scene.camera': {}} : {'xaxis.autorange': true, 'yaxis.autorange': true});
    }
};
$('clear').onclick = () => { layout = []; persist(); draw(); subscribe(); };
$('save').onclick = () => {
    const url = URL.createObjectURL(new Blob([JSON.stringify(layout, null, 2)], {type: 'application/json'}));
    const anchor = document.createElement('a'); anchor.href = url; anchor.download = 'cubesat-layout.json'; anchor.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
};
$('load').onclick = () => $('file').click();
$('file').onchange = async () => {
    try {
        const imported = validate(JSON.parse(await $('file').files[0].text()));
        layout = []; draw(); layout = imported; persist(); draw(); subscribe();
    } catch (error) { $('status').textContent = 'Cannot import layout: ' + error.message; }
    $('file').value = '';
};
(async () => {
    if (typeof Plotly === 'undefined') throw Error('Plotly failed to load. Install the updated requirements and restart.');
    const response = await fetch('/api/catalog'); if (!response.ok) throw Error('Cannot load graph catalog');
    catalog = await response.json();
    try {
        const stored = localStorage.getItem(storageKey);
        layout = validate(stored ? JSON.parse(stored) : [{id: 'body:3'}, {id: 'battery:0'}, {id: 'torquers:0'}, {id: 'boom:0'}]);
    } catch (_) { layout = []; }
    draw(); connect();
})().catch(error => { $('status').textContent = error.message; });
