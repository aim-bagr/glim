// GLIM Headless SLAM Dashboard & Real-Time Three.js Visualizer
let ws = null;
let currentJobState = 'idle';
let currentRunName = null;
let autoScroll = true;
let totalLogLines = 0;
let liveStreamTimer = null;

// Three.js State
let threeScene, threeCamera, threeRenderer, threeControls;
let currentRunSubmaps = [];
let submapCache = new Map(); // submap_id -> THREE.Points
let threeTrajectoryLine = null;
let activeSubmapId = null;
let isIsolated = false;
let colorMode = 'rainbow';
let pointSize = 2.0;

// DOM Elements
const statusBadge = document.getElementById('service-status-badge');
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const wsIndicator = document.getElementById('ws-indicator');
const wsText = document.getElementById('ws-text');

// Floating Modals & Drawers
const launcherModal = document.getElementById('launcher-modal');
const runsModal = document.getElementById('runs-modal');
const consoleDrawer = document.getElementById('console-drawer');
const minConsoleBtn = document.getElementById('min-console-btn');

const toggleLauncherBtn = document.getElementById('toggle-launcher-btn');
const closeLauncherBtn = document.getElementById('close-launcher-btn');
const toggleRunsBtn = document.getElementById('toggle-runs-btn');
const closeRunsBtn = document.getElementById('close-runs-btn');
const toggleConsoleBtn = document.getElementById('toggle-console-btn');
const closeConsoleBtn = document.getElementById('close-console-btn');

// Launcher Form
const launcherForm = document.getElementById('launcher-form');
const datasetSelect = document.getElementById('dataset-select');
const runNameInput = document.getElementById('run-name-input');
const startBtn = document.getElementById('start-btn');
const headerStopBtn = document.getElementById('header-stop-btn');
const runsList = document.getElementById('runs-list');

// Log Console
const logConsole = document.getElementById('log-console');
const logCount = document.getElementById('log-count');
const autoScrollToggle = document.getElementById('autoscroll-toggle');
const clearLogsBtn = document.getElementById('clear-logs-btn');

// Top KPIs
const metricBagTime = document.getElementById('metric-bag-time');
const metricSpeed = document.getElementById('metric-speed');
const metricScans = document.getElementById('metric-scans');
const metricQueueOdom = document.getElementById('metric-queue-odom');
const metricQueueSub = document.getElementById('metric-queue-sub');
const metricQueueGlob = document.getElementById('metric-queue-glob');

// 3D Controls Elements
const viewportContainer = document.getElementById('viewport-3d');
const viewportPlaceholder = document.getElementById('viewport-placeholder');
const viewerRunSelect = document.getElementById('viewer-run-select');
const viewerLoadBtn = document.getElementById('viewer-load-btn');
const viewerIsolateToggle = document.getElementById('viewer-isolate-toggle');
const viewerPrevBtn = document.getElementById('viewer-prev-btn');
const viewerNextBtn = document.getElementById('viewer-next-btn');
const viewerSubmapSlider = document.getElementById('viewer-submap-slider');
const viewerFocusBtn = document.getElementById('viewer-focus-btn');
const viewerColorMode = document.getElementById('viewer-color-mode');
const viewerPointSize = document.getElementById('viewer-point-size');
const viewerHud = document.getElementById('viewer-hud');
const hudSubmapText = document.getElementById('hud-submap-text');

// Initialize Application
document.addEventListener('DOMContentLoaded', () => {
  if (window.lucide) lucide.createIcons();

  initThreeJS();
  fetchDatasets();
  fetchRuns();
  connectWebSocket();
  setupModalToggles();
  setup3DViewerControls();

  // Launcher & Actions
  launcherForm.addEventListener('submit', handleStartJob);
  headerStopBtn.addEventListener('click', handleStopJob);
  document.getElementById('refresh-datasets-btn').addEventListener('click', fetchDatasets);
  document.getElementById('refresh-runs-btn').addEventListener('click', fetchRuns);

  clearLogsBtn.addEventListener('click', () => {
    logConsole.innerHTML = '';
    totalLogLines = 0;
    updateLogCount();
  });

  autoScrollToggle.addEventListener('change', (e) => {
    autoScroll = e.target.checked;
  });
});

// Setup Floating Modal Visibility & Toggles
function setupModalToggles() {
  toggleLauncherBtn.addEventListener('click', () => {
    const isHidden = launcherModal.classList.contains('hidden');
    if (isHidden) {
      runsModal.classList.add('hidden'); // Close runs if opening launcher
      launcherModal.classList.remove('hidden');
    } else {
      launcherModal.classList.add('hidden');
    }
  });

  closeLauncherBtn.addEventListener('click', () => {
    launcherModal.classList.add('hidden');
  });

  toggleRunsBtn.addEventListener('click', () => {
    const isHidden = runsModal.classList.contains('hidden');
    if (isHidden) {
      launcherModal.classList.add('hidden');
      runsModal.classList.remove('hidden');
      fetchRuns();
    } else {
      runsModal.classList.add('hidden');
    }
  });

  closeRunsBtn.addEventListener('click', () => {
    runsModal.classList.add('hidden');
  });

  toggleConsoleBtn.addEventListener('click', () => {
    consoleDrawer.classList.toggle('hidden');
    minConsoleBtn.classList.toggle('hidden', !consoleDrawer.classList.contains('hidden'));
  });

  closeConsoleBtn.addEventListener('click', () => {
    consoleDrawer.classList.add('hidden');
    minConsoleBtn.classList.remove('hidden');
  });

  minConsoleBtn.addEventListener('click', () => {
    consoleDrawer.classList.remove('hidden');
    minConsoleBtn.classList.add('hidden');
  });
}

// WebSocket Telemetry Connection
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/api/ws`;

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    wsIndicator.className = 'flex items-center gap-1.5 text-xs text-emerald-400';
    wsText.textContent = 'Live';
  };

  ws.onclose = () => {
    wsIndicator.className = 'flex items-center gap-1.5 text-xs text-slate-500';
    wsText.textContent = 'Offline';
    setTimeout(connectWebSocket, 2000);
  };

  ws.onerror = () => {
    ws.close();
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === 'status') {
        updateJobStatus(msg.data);
      } else if (msg.type === 'log') {
        appendLog(msg.line);
        if (msg.progress) {
          updateJobStatus(msg.progress);
        }
      }
    } catch (e) {
      console.error('WS message error:', e);
    }
  };
}

// Update Job Status & Top KPI Strip
function updateJobStatus(job) {
  currentJobState = job.state;
  statusText.textContent = job.state.toUpperCase();

  // Reset Dot animations
  statusDot.className = 'h-2 w-2 rounded-full';

  switch (job.state) {
    case 'running':
      statusBadge.className = 'flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-medium border bg-amber-500/10 border-amber-500/30 text-amber-400';
      statusDot.classList.add('bg-amber-400', 'status-running');
      headerStopBtn.classList.remove('hidden');
      startBtn.disabled = true;

      // Start live streaming active submaps & trajectory if not already started
      if (job.run_name && job.run_name !== currentRunName) {
        startLiveTracking(job.run_name);
      }
      break;

    case 'finalizing':
      statusBadge.className = 'flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-medium border bg-purple-500/10 border-purple-500/30 text-purple-400';
      statusDot.classList.add('bg-purple-400', 'status-running');
      headerStopBtn.classList.remove('hidden');
      break;

    case 'completed':
      statusBadge.className = 'flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-medium border bg-emerald-500/10 border-emerald-500/30 text-emerald-400';
      statusDot.classList.add('bg-emerald-400');
      headerStopBtn.classList.add('hidden');
      startBtn.disabled = false;
      startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
      if (window.lucide) lucide.createIcons();
      stopLiveTracking(true);
      fetchRuns();
      break;

    case 'stopped':
    case 'failed':
      statusBadge.className = 'flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-medium border bg-rose-500/10 border-rose-500/30 text-rose-400';
      statusDot.classList.add('bg-rose-400');
      headerStopBtn.classList.add('hidden');
      startBtn.disabled = false;
      startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
      if (window.lucide) lucide.createIcons();
      stopLiveTracking(false);
      fetchRuns();
      break;

    default: // idle
      statusBadge.className = 'flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-medium border bg-slate-900 border-slate-800 text-slate-300';
      statusDot.classList.add('bg-slate-500');
      headerStopBtn.classList.add('hidden');
      startBtn.disabled = false;
      startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
      if (window.lucide) lucide.createIcons();
      break;
  }

  // Update KPI counters
  metricSpeed.textContent = job.speed ? `${job.speed.toFixed(1)}x` : '0.0x';
  metricBagTime.textContent = job.bag_time ? `${job.bag_time.toFixed(1)}s` : '0.0s';
  metricScans.textContent = job.scans || '0';
  metricQueueOdom.textContent = job.queue_odom || '0';
  metricQueueSub.textContent = job.queue_sub || '0';
  metricQueueGlob.textContent = job.queue_glob || '0';
}

// Append Terminal Log
function appendLog(line) {
  const div = document.createElement('div');
  div.className = 'log-line';

  if (line.includes('[error]') || line.includes('critical') || line.includes('failed')) {
    div.classList.add('log-err');
  } else if (line.includes('[warning]')) {
    div.classList.add('log-warn');
  } else if (line.includes('[progress]')) {
    div.classList.add('log-progress');
  } else {
    div.classList.add('log-info');
  }

  div.textContent = line;
  logConsole.appendChild(div);
  totalLogLines++;
  updateLogCount();

  if (autoScroll) {
    logConsole.scrollTop = logConsole.scrollHeight;
  }
}

function updateLogCount() {
  logCount.textContent = `${totalLogLines} lines`;
}

// Fetch Available Datasets
async function fetchDatasets() {
  try {
    const res = await fetch('/api/datasets');
    const datasets = await res.json();
    datasetSelect.innerHTML = '';

    if (datasets.length === 0) {
      datasetSelect.innerHTML = '<option value="">No MCAP datasets found in /data</option>';
      return;
    }

    datasets.forEach(d => {
      const opt = document.createElement('option');
      opt.value = d.path;
      opt.textContent = `${d.name} (${d.size_human})`;
      datasetSelect.appendChild(opt);
    });
  } catch (err) {
    datasetSelect.innerHTML = '<option value="">Error scanning datasets</option>';
  }
}

// Fetch Past Runs History
async function fetchRuns() {
  try {
    const res = await fetch('/api/runs');
    const runs = await res.json();
    runsList.innerHTML = '';

    // Update 3D Run Select Dropdown
    const prevSelectVal = viewerRunSelect.value;
    viewerRunSelect.innerHTML = '<option value="">Select a run...</option>';

    if (runs.length === 0) {
      runsList.innerHTML = '<div class="text-xs text-slate-500 py-4 text-center">No past runs found in /data/glim_results</div>';
      return;
    }

    runs.forEach(run => {
      // Add to 3D Dropdown
      const opt = document.createElement('option');
      opt.value = run.name;
      opt.textContent = `${run.name} (${run.submaps_count} submaps)`;
      viewerRunSelect.appendChild(opt);

      // Add to Past Runs Modal
      const item = document.createElement('div');
      item.className = 'bg-slate-950/70 border border-slate-800/80 rounded-lg p-2.5 hover:border-slate-700 transition flex items-center justify-between gap-3 text-xs';
      item.innerHTML = `
        <div class="min-w-0">
          <div class="font-semibold text-slate-200 truncate">${run.name}</div>
          <div class="text-[11px] text-slate-400 flex items-center gap-1.5 mt-0.5">
            <span>${run.modified_at}</span>
            <span>•</span>
            <span class="text-blue-400 font-medium">${run.submaps_count} submaps</span>
            <span>•</span>
            <span>${run.size_human}</span>
          </div>
        </div>
        <div class="flex items-center gap-1 shrink-0">
          <button onclick="openRunIn3D('${run.name}')" class="p-1.5 bg-blue-600/20 hover:bg-blue-600/30 text-blue-400 rounded flex items-center gap-1 transition" title="View in 3D">
            <i data-lucide="box" class="w-3.5 h-3.5"></i>
            <span class="text-[11px] font-medium hidden sm:inline">3D</span>
          </button>
          ${run.has_traj_lidar ? `
            <a href="/api/runs/${encodeURIComponent(run.name)}/trajectory?opt=true" download="${run.name}_traj_lidar.txt" class="p-1.5 bg-slate-800 hover:bg-slate-700 text-slate-300 rounded flex items-center transition" title="Download Optimized LiDAR Trajectory (TUM)">
              <i data-lucide="download" class="w-3.5 h-3.5"></i>
            </a>` : ''}
        </div>
      `;
      runsList.appendChild(item);
    });

    if (window.lucide) lucide.createIcons();

    if (prevSelectVal) {
      viewerRunSelect.value = prevSelectVal;
    } else if (runs.length > 0 && !currentRunName) {
      viewerRunSelect.value = runs[0].name;
    }
  } catch (err) {
    runsList.innerHTML = '<div class="text-xs text-rose-500 py-4 text-center">Failed to load run history</div>';
  }
}

// Open run directly in 3D
window.openRunIn3D = function(runName) {
  runsModal.classList.add('hidden');
  viewerRunSelect.value = runName;
  loadMapRun(runName);
};

// Start SLAM Job
async function handleStartJob(e) {
  e.preventDefault();
  const datasetPath = datasetSelect.value;
  if (!datasetPath) {
    alert('Please select an input MCAP dataset.');
    return;
  }

  const payload = {
    dataset_path: datasetPath,
    run_name: runNameInput.value.trim() || null,
    max_num_keyframes: parseInt(document.getElementById('param-keyframes').value, 10),
    submap_target_num_points: parseInt(document.getElementById('param-points').value, 10),
    min_travel_dist: parseFloat(document.getElementById('param-min-travel').value),
    start_offset_sec: parseFloat(document.getElementById('param-start-offset').value)
  };

  startBtn.disabled = true;
  startBtn.textContent = 'Launching...';

  try {
    const res = await fetch('/api/jobs/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
    if (!res.ok) {
      const err = await res.json();
      throw new Error(err.detail || 'Failed to start job');
    }
    const job = await res.json();

    // Auto-close launcher & open console for visibility
    launcherModal.classList.add('hidden');
    consoleDrawer.classList.remove('hidden');
    minConsoleBtn.classList.add('hidden');

    updateJobStatus(job);
    startLiveTracking(job.run_name);
  } catch (err) {
    alert(`Could not start job: ${err.message}`);
    startBtn.disabled = false;
    startBtn.textContent = 'Start SLAM Run';
  }
}

// Stop SLAM Job
async function handleStopJob() {
  if (!confirm('Are you sure you want to stop and finalize the current SLAM run?')) return;
  headerStopBtn.disabled = true;
  headerStopBtn.textContent = 'Finalizing...';

  try {
    const res = await fetch('/api/jobs/stop', { method: 'POST' });
    const job = await res.json();
    updateJobStatus(job);
  } catch (err) {
    alert(`Error stopping job: ${err.message}`);
  } finally {
    headerStopBtn.disabled = false;
  }
}

// ============================================================================
// REAL-TIME 3D VIEWPORT & SUBMAP STREAMING (Three.js)
// ============================================================================

function initThreeJS() {
  const container = viewportContainer;
  const width = window.innerWidth;
  const height = window.innerHeight;

  // Scene
  threeScene = new THREE.Scene();
  threeScene.background = new THREE.Color(0x020617); // Slate-950

  // Camera (Z-Up for SLAM / Robotics convention)
  threeCamera = new THREE.PerspectiveCamera(55, width / height, 0.1, 5000);
  threeCamera.up.set(0, 0, 1);
  threeCamera.position.set(-25, -25, 20);

  // Renderer
  threeRenderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  threeRenderer.setSize(width, height);
  threeRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.appendChild(threeRenderer.domElement);

  // OrbitControls
  threeControls = new THREE.OrbitControls(threeCamera, threeRenderer.domElement);
  threeControls.enableDamping = true;
  threeControls.dampingFactor = 0.08;
  threeControls.screenSpacePanning = true;
  threeControls.target.set(0, 0, 0);

  // Grid Helper (aligned with Z-up)
  const grid = new THREE.GridHelper(200, 40, 0x1e293b, 0x0f172a);
  grid.rotation.x = Math.PI / 2;
  threeScene.add(grid);

  // Axes Helper (X: Red, Y: Green, Z: Blue)
  const axes = new THREE.AxesHelper(3.0);
  threeScene.add(axes);

  // Window Resize Listener
  window.addEventListener('resize', onWindowResize);

  // Animation Loop
  function animate() {
    requestAnimationFrame(animate);
    threeControls.update();
    threeRenderer.render(threeScene, threeCamera);
  }
  animate();
}

function onWindowResize() {
  if (!threeCamera || !threeRenderer) return;
  const w = window.innerWidth;
  const h = window.innerHeight;
  threeCamera.aspect = w / h;
  threeCamera.updateProjectionMatrix();
  threeRenderer.setSize(w, h);
}

// 3D HUD Controls Setup
function setup3DViewerControls() {
  viewerLoadBtn.addEventListener('click', () => {
    const runName = viewerRunSelect.value;
    if (runName) loadMapRun(runName);
  });

  viewerIsolateToggle.addEventListener('change', (e) => {
    isIsolated = e.target.checked;
    updateSubmapVisibility();
  });

  viewerSubmapSlider.addEventListener('input', (e) => {
    activeSubmapId = parseInt(e.target.value, 10);
    onActiveSubmapChanged(activeSubmapId);
  });

  viewerPrevBtn.addEventListener('click', () => {
    if (activeSubmapId > 0) {
      activeSubmapId--;
      viewerSubmapSlider.value = activeSubmapId;
      onActiveSubmapChanged(activeSubmapId);
    }
  });

  viewerNextBtn.addEventListener('click', () => {
    if (activeSubmapId < currentRunSubmaps.length - 1) {
      activeSubmapId++;
      viewerSubmapSlider.value = activeSubmapId;
      onActiveSubmapChanged(activeSubmapId);
    }
  });

  viewerFocusBtn.addEventListener('click', () => {
    if (activeSubmapId !== null) {
      focusCameraOnSubmap(activeSubmapId);
    }
  });

  viewerColorMode.addEventListener('change', (e) => {
    colorMode = e.target.value;
    submapCache.forEach((points, id) => {
      applySubmapColors(points.geometry, id);
    });
  });

  viewerPointSize.addEventListener('input', (e) => {
    pointSize = parseFloat(e.target.value);
    submapCache.forEach((points) => {
      points.material.size = pointSize;
    });
  });
}

// Clear Scene Between Runs
function clear3DScene() {
  submapCache.forEach((points) => {
    threeScene.remove(points);
    if (points.geometry) points.geometry.dispose();
    if (points.material) points.material.dispose();
  });
  submapCache.clear();

  if (threeTrajectoryLine) {
    threeScene.remove(threeTrajectoryLine);
    if (threeTrajectoryLine.geometry) threeTrajectoryLine.geometry.dispose();
    if (threeTrajectoryLine.material) threeTrajectoryLine.material.dispose();
    threeTrajectoryLine = null;
  }

  currentRunSubmaps = [];
  activeSubmapId = null;
  viewerSubmapSlider.max = 0;
  viewerSubmapSlider.value = 0;
  viewerHud.classList.add('hidden');
}

// Load a Completed or Past Run
async function loadMapRun(runName) {
  if (!runName) return;
  currentRunName = runName;
  viewportPlaceholder.classList.add('hidden');
  clear3DScene();

  viewerLoadBtn.disabled = true;
  viewerLoadBtn.textContent = 'Loading...';

  try {
    // 1. Fetch Trajectory Binary
    await loadTrajectory(runName, true);

    // 2. Fetch Submaps Metadata
    const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/submaps`);
    if (!res.ok) throw new Error('Failed to load submap list');
    currentRunSubmaps = await res.json();

    if (currentRunSubmaps.length === 0) {
      hudSubmapText.textContent = 'No submaps recorded';
      viewerHud.classList.remove('hidden');
      return;
    }

    viewerSubmapSlider.max = currentRunSubmaps.length - 1;
    activeSubmapId = 0;
    viewerSubmapSlider.value = 0;
    viewerHud.classList.remove('hidden');

    // 3. Load First Batch of Submaps
    const initialBatch = currentRunSubmaps.slice(0, 30);
    await Promise.all(initialBatch.map(sm => loadSubmapPoints(runName, sm.id)));

    onActiveSubmapChanged(0);
    focusCameraOnSubmap(0);

    // Stream remaining submaps in background
    if (currentRunSubmaps.length > 30) {
      streamRemainingSubmaps(runName, 30);
    }
  } catch (err) {
    console.error('Failed to load map run:', err);
    alert(`Could not load map: ${err.message}`);
  } finally {
    viewerLoadBtn.disabled = false;
    viewerLoadBtn.textContent = 'Load';
  }
}

// Background Submap Streaming
async function streamRemainingSubmaps(runName, startIndex) {
  const chunkSize = 15;
  for (let i = startIndex; i < currentRunSubmaps.length; i += chunkSize) {
    if (currentRunName !== runName) break;
    const chunk = currentRunSubmaps.slice(i, i + chunkSize);
    await Promise.all(chunk.map(sm => loadSubmapPoints(runName, sm.id)));
    await new Promise(r => setTimeout(r, 40));
  }
}

// Load Binary Submap Point Cloud (Zero deserialization overhead)
async function loadSubmapPoints(runName, submapId) {
  if (submapCache.has(submapId)) {
    return submapCache.get(submapId);
  }

  const sm = currentRunSubmaps.find(s => s.id === submapId);
  if (!sm) return null;

  try {
    const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/submaps/${submapId}/points`);
    if (!res.ok) return null;
    const buffer = await res.arrayBuffer();

    const positions = new Float32Array(buffer);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    // Apply submap origin transformation matrix
    const matrix = new THREE.Matrix4();
    matrix.fromArray(sm.matrix);
    geom.applyMatrix4(matrix);

    // Apply Altitude Rainbow Colors
    applySubmapColors(geom, submapId);

    const mat = new THREE.PointsMaterial({
      size: pointSize,
      vertexColors: true,
      sizeAttenuation: true
    });

    const pointsObj = new THREE.Points(geom, mat);
    threeScene.add(pointsObj);
    submapCache.set(submapId, pointsObj);

    if (isIsolated) {
      pointsObj.visible = (submapId === activeSubmapId);
    }

    return pointsObj;
  } catch (err) {
    return null;
  }
}

// Load Binary Trajectory
async function loadTrajectory(runName, opt = true) {
  try {
    const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/trajectory_binary?opt=${opt}`);
    if (!res.ok) return;
    const buffer = await res.arrayBuffer();
    const positions = new Float32Array(buffer);

    if (threeTrajectoryLine) {
      threeTrajectoryLine.geometry.dispose();
      threeTrajectoryLine.geometry = new THREE.BufferGeometry();
      threeTrajectoryLine.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    } else {
      const geom = new THREE.BufferGeometry();
      geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      const mat = new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 2 });
      threeTrajectoryLine = new THREE.Line(geom, mat);
      threeScene.add(threeTrajectoryLine);
    }
  } catch (err) {
    console.warn('Trajectory fetch failed:', err);
  }
}

// Altitude Rainbow Color Calculation (Turbo Colormap)
function applySubmapColors(geom, submapId) {
  const pos = geom.attributes.position;
  const count = pos.count;
  const colors = new Float32Array(count * 3);

  let minZ = Infinity, maxZ = -Infinity;
  for (let i = 0; i < count; i++) {
    const z = pos.getZ(i);
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
  }
  const rangeZ = Math.max(maxZ - minZ, 0.1);

  for (let i = 0; i < count; i++) {
    if (colorMode === 'flat') {
      colors[i * 3] = 0.92;
      colors[i * 3 + 1] = 0.92;
      colors[i * 3 + 2] = 0.92;
    } else {
      const t = (pos.getZ(i) - minZ) / rangeZ;
      const rgb = turboColormap(t);
      colors[i * 3] = rgb[0];
      colors[i * 3 + 1] = rgb[1];
      colors[i * 3 + 2] = rgb[2];
    }
  }

  geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geom.attributes.color.needsUpdate = true;
}

function turboColormap(t) {
  const r = Math.sin(t * Math.PI * 1.5);
  const g = Math.sin(t * Math.PI);
  const b = Math.cos(t * Math.PI * 1.5);
  return [
    Math.max(0, Math.min(1, r * 0.8 + 0.2)),
    Math.max(0, Math.min(1, g)),
    Math.max(0, Math.min(1, b * 0.9 + 0.1))
  ];
}

// Active Submap Selection Changed
function onActiveSubmapChanged(submapId) {
  const sm = currentRunSubmaps.find(s => s.id === submapId);
  if (sm) {
    hudSubmapText.textContent = `Submap #${String(submapId).padStart(4, '0')} (${sm.num_points.toLocaleString()} pts)`;
  } else {
    hudSubmapText.textContent = `Submap #${submapId}`;
  }
  updateSubmapVisibility();
  loadSubmapPoints(currentRunName, submapId);
}

// Update Submap Visibility for Isolation Mode
function updateSubmapVisibility() {
  submapCache.forEach((points, id) => {
    if (isIsolated) {
      points.visible = (id === activeSubmapId);
    } else {
      points.visible = true;
    }
  });
}

// Focus Camera smoothly on Submap Position
function focusCameraOnSubmap(submapId) {
  const sm = currentRunSubmaps.find(s => s.id === submapId);
  if (!sm || !threeControls) return;

  const targetPos = new THREE.Vector3(sm.pos[0], sm.pos[1], sm.pos[2]);
  threeControls.target.copy(targetPos);
  threeCamera.position.set(sm.pos[0] - 18, sm.pos[1] - 18, sm.pos[2] + 14);
  threeControls.update();
}

// ============================================================================
// STRETCH GOAL: LIVE STREAMING SUBMAPS & TRAJECTORY DURING REPLAY
// ============================================================================

function startLiveTracking(runName) {
  stopLiveTracking(false);
  currentRunName = runName;
  viewportPlaceholder.classList.add('hidden');
  viewerHud.classList.remove('hidden');

  let knownSubmapCount = 0;

  async function pollLiveRun() {
    if (currentJobState !== 'running' && currentJobState !== 'finalizing') {
      stopLiveTracking(true);
      return;
    }

    try {
      // 1. Poll Trajectory live
      await loadTrajectory(runName, false);

      // 2. Poll Submaps list live
      const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/submaps`);
      if (res.ok) {
        const submaps = await res.json();
        currentRunSubmaps = submaps;

        if (submaps.length > knownSubmapCount) {
          viewerSubmapSlider.max = Math.max(0, submaps.length - 1);

          // Ingest new submaps
          for (let i = knownSubmapCount; i < submaps.length; i++) {
            await loadSubmapPoints(runName, submaps[i].id);
          }

          // Follow the latest submap if camera is at origin
          const latestId = submaps[submaps.length - 1].id;
          if (activeSubmapId === null || knownSubmapCount === 0) {
            activeSubmapId = latestId;
            viewerSubmapSlider.value = latestId;
            focusCameraOnSubmap(latestId);
          }
          hudSubmapText.textContent = `Live Submap #${latestId} (${submaps.length} total)`;
          knownSubmapCount = submaps.length;
        }
      }
    } catch (e) {
      console.warn('Live tracking poll error:', e);
    }
  }

  // Poll every 1.5 seconds while job runs
  pollLiveRun();
  liveStreamTimer = setInterval(pollLiveRun, 1500);
}

function stopLiveTracking(loadFinalOptimized = true) {
  if (liveStreamTimer) {
    clearInterval(liveStreamTimer);
    liveStreamTimer = null;
  }
  if (loadFinalOptimized && currentRunName) {
    // Reload final loop-closed trajectory and submap poses
    setTimeout(() => {
      loadMapRun(currentRunName);
    }, 1000);
  }
}
