const canvas = document.getElementById('graphCanvas');
const ctx = canvas.getContext('2d');
const metaEl = document.getElementById('meta');
const statusEl = document.getElementById('status');
const refreshBtn = document.getElementById('refreshBtn');
let lastGraph = null;

function clipLabel(text, maxLen = 24) {
  if (!text) {
    return '';
  }
  return text.length > maxLen ? text.slice(0, maxLen - 1) + '…' : text;
}

function resizeCanvas() {
  const headerHeight = document.querySelector('header').offsetHeight;
  canvas.width = window.innerWidth;
  canvas.height = Math.max(240, window.innerHeight - headerHeight - 40);
  renderGraph(lastGraph);
}

window.addEventListener('resize', resizeCanvas);
refreshBtn.addEventListener('click', fetchGraph);

function renderGraph(graph) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!graph) {
    return;
  }
  lastGraph = graph;
  const width = canvas.width;
  const height = canvas.height;
  const nodeNames = graph.nodes || [];
  const topicNames = Object.keys(graph.topics || {});

  const positions = {};
  const nodeY = height * 0.3;
  const topicY = height * 0.7;

  function allocate(names, y) {
    const count = names.length;
    names.forEach((name, idx) => {
      const x = count > 1 ? ((idx + 1) / (count + 1)) * width : width / 2;
      positions[name] = { x, y };
    });
  }

  allocate(nodeNames, nodeY);
  allocate(topicNames, topicY);

  function getPos(name) {
    if (!positions[name]) {
      positions[name] = { x: width / 2, y: height / 2 };
    }
    return positions[name];
  }

  ctx.lineWidth = 1.5;
  ctx.strokeStyle = '#8b949e';
  ctx.fillStyle = '#8b949e';
  (graph.edges || []).forEach(edge => {
    const start = getPos(edge.start);
    const end = getPos(edge.end);
    drawArrow(start, end);
    if (edge.qos) {
      const midX = (start.x + end.x) / 2;
      const midY = (start.y + end.y) / 2;
      ctx.save();
      ctx.fillStyle = '#9e6ffe';
      ctx.font = '11px monospace';
      ctx.fillText(edge.qos, midX + 4, midY - 2);
      ctx.restore();
    }
  });

  nodeNames.forEach(name => drawNode(name, positions[name], '#58a6ff'));
  topicNames.forEach(name => drawTopic(name, positions[name], '#3fb950'));
}

function drawArrow(start, end) {
  ctx.beginPath();
  ctx.moveTo(start.x, start.y);
  ctx.lineTo(end.x, end.y);
  ctx.stroke();
  const angle = Math.atan2(end.y - start.y, end.x - start.x);
  const headLen = 9;
  ctx.beginPath();
  ctx.moveTo(end.x, end.y);
  ctx.lineTo(end.x - headLen * Math.cos(angle - Math.PI / 6), end.y - headLen * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(end.x - headLen * Math.cos(angle + Math.PI / 6), end.y - headLen * Math.sin(angle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();
}

function drawNode(name, pos, color) {
  ctx.save();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(pos.x, pos.y, 14, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#0d1117';
  ctx.font = '11px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(clipLabel(name), pos.x, pos.y - 18);
  ctx.restore();
}

function drawTopic(name, pos, color) {
  ctx.save();
  ctx.fillStyle = color;
  ctx.fillRect(pos.x - 18, pos.y - 12, 36, 24);
  ctx.fillStyle = '#0d1117';
  ctx.font = '11px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(clipLabel(name), pos.x, pos.y - 18);
  ctx.restore();
}

async function fetchGraph() {
  refreshBtn.disabled = true;
  try {
    const response = await fetch('/graph?ts=' + Date.now(), { cache: 'no-store' });
    if (!response.ok) {
      throw new Error('HTTP ' + response.status);
    }
    const payload = await response.json();
    const graph = payload.graph;
    const nodes = graph.nodes?.length ?? 0;
    const topics = Object.keys(graph.topics || {}).length;
    const edges = graph.edges?.length ?? 0;
    metaEl.textContent =
      'nodes: ' + nodes +
      ' | topics: ' + topics +
      ' | edges: ' + edges +
      ' | fingerprint: ' + payload.fingerprint;
    statusEl.textContent = 'Last update: ' + new Date(payload.generated_at * 1000).toLocaleTimeString();
    renderGraph(graph);
  } catch (err) {
    statusEl.textContent = 'Error fetching data: ' + err.message;
  } finally {
    refreshBtn.disabled = false;
  }
}

resizeCanvas();
fetchGraph();
setInterval(fetchGraph, 2000);
