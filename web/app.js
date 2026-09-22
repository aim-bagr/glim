// State
let ws = null;
let autoScroll = true;
let isJobRunning = false;

// DOM Elements
const statusBadge = document.getElementById('service-status-badge');
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const wsIndicator = document.getElementById('ws-indicator');
const wsText = document.getElementById('ws-text');

const datasetSelect = document.getElementById('dataset-select');
const runNameInput = document.getElementById('run-name-input');
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const launcherForm = document.getElementById('launcher-form');
const runsList = document.getElementById('runs-list');
const logConsole = document.getElementById('log-console');
const autoScrollToggle = document.getElementById('autoscroll-toggle');

const metricBagTime = document.getElementById('metric-bag-time');
const metricSpeed = document.getElementById('metric-speed');
const metricScans = document.getElementById('metric-scans');
const metricElapsed = document.getElementById('metric-elapsed');
const metricQueueOdom = document.getElementById('metric-queue-odom');
const metricQueueSub = document.getElementById('metric-queue-sub');
const metricQueueGlob = document.getElementById('metric-queue-glob');

// Initialize
document.addEventListener('DOMContentLoaded', () => {
  if (window.lucide) lucide.createIcons();
  fetchDatasets();
  fetchRuns();
  connectWebSocket();

  // Listeners
  launcherForm.addEventListener('submit', handleStartJob);
  stopBtn.addEventListener('click', handleStopJob);
  document.getElementById('refresh-datasets-btn').addEventListener('click', fetchDatasets);
  document.getElementById('refresh-runs-btn').addEventListener('click', fetchRuns);
  document.getElementById('clear-logs-btn').addEventListener('click', () => {
    logConsole.innerHTML = '';
  });
  autoScrollToggle.addEventListener('change', (e) => {
    autoScroll = e.target.checked;
  });
});

// WebSocket Connection
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
    wsText.textContent = 'Reconnecting...';
    setTimeout(connectWebSocket, 2000);
  };

  ws.onerror = () => {
    ws.close();
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === 'status') {
        updateJobState(msg.data);
      } else if (msg.type === 'log') {
        appendLog(msg.line);
        if (msg.progress) {
          updateMetrics(msg.progress);
        }
      }
    } catch (e) {
      console.error('WS Parse Error', e);
    }
  };
}

// Fetch Datasets
async function fetchDatasets() {
  try {
    const res = await fetch('/api/datasets');
    const datasets = await res.json();
    datasetSelect.innerHTML = '';

    if (!datasets || datasets.length === 0) {
      datasetSelect.innerHTML = '<option value="">No .mcap datasets found in /data</option>';
      return;
    }

    datasets.forEach(ds => {
      const opt = document.createElement('option');
      opt.value = ds.path;
      opt.textContent = `${ds.name} (${ds.size_human})`;
      datasetSelect.appendChild(opt);
    });

    // Auto-generate suggested run name
    if (datasets.length > 0 && !runNameInput.value) {
      const baseName = datasets[0].name.replace(/\.[^/.]+$/, '').replace(/[^a-zA-Z0-9_-]/g, '_');
      runNameInput.placeholder = `run_${baseName.substring(0, 16)}`;
    }
  } catch (err) {
    datasetSelect.innerHTML = '<option value="">Failed to fetch datasets</option>';
  }
}

// Fetch Historical Runs
async function fetchRuns() {
  try {
    const res = await fetch('/api/runs');
    const runs = await res.json();
    runsList.innerHTML = '';

    if (!runs || runs.length === 0) {
      runsList.innerHTML = '<div class="text-xs text-slate-500 py-4 text-center">No completed runs found yet.</div>';
      return;
    }

    runs.forEach(run => {
      const item = document.createElement('div');
      item.className = 'bg-slate-950/70 border border-slate-800/80 rounded-lg p-3 hover:border-slate-700 transition flex items-center justify-between gap-3';
      item.innerHTML = `
        <div class="min-w-0">
          <div class="text-xs font-semibold text-slate-200 truncate">${run.name}</div>
          <div class="text-[11px] text-slate-400 flex items-center gap-2 mt-0.5">
            <span>${run.modified_at}</span>
            <span>•</span>
            <span class="text-blue-400 font-medium">${run.submaps_count} submaps</span>
            <span>•</span>
            <span>${run.size_human}</span>
          </div>
        </div>
        <div class="flex items-center gap-1.5 shrink-0">
          ${run.has_traj_lidar ? `
            <a href="/api/runs/${encodeURIComponent(run.name)}/trajectory?opt=true" download="${run.name}_traj_lidar.txt" class="p-1.5 bg-blue-600/20 hover:bg-blue-600/30 text-blue-400 rounded text-xs flex items-center gap-1 transition" title="Download Optimized LiDAR Trajectory (TUM)">
              <i data-lucide="download" class="w-3.5 h-3.5"></i>
              <span class="text-[11px] font-medium hidden sm:inline">Traj</span>
            </a>` : ''}
          ${run.has_trajectory_tum ? `
            <a href="/api/runs/${encodeURIComponent(run.name)}/trajectory?opt=false" download="${run.name}_odom_tum.txt" class="p-1.5 bg-purple-600/20 hover:bg-purple-600/30 text-purple-400 rounded text-xs flex items-center gap-1 transition" title="Download Odometry Trajectory (TUM)">
              <i data-lucide="file-text" class="w-3.5 h-3.5"></i>
              <span class="text-[11px] font-medium hidden sm:inline">Odom</span>
            </a>` : ''}
        </div>
      `;
      runsList.appendChild(item);
    });
    if (window.lucide) lucide.createIcons();
  } catch (err) {
    runsList.innerHTML = '<div class="text-xs text-rose-500 py-4 text-center">Failed to load run history</div>';
  }
}

// Start SLAM Job
async function handleStartJob(e) {
  e.preventDefault();
  const datasetPath = datasetSelect.value;
  if (!datasetPath) {
    alert('Please select a dataset.');
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
    updateJobState(job);
  } catch (err) {
    alert(err.message);
    startBtn.disabled = false;
    startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
    if (window.lucide) lucide.createIcons();
  }
}

// Stop SLAM Job
async function handleStopJob() {
  if (!confirm('Stop current SLAM run? Final submaps and trajectory will be flushed to disk.')) {
    return;
  }
  stopBtn.disabled = true;
  stopBtn.textContent = 'Finalizing...';
  try {
    await fetch('/api/jobs/stop', { method: 'POST' });
  } catch (err) {
    alert('Error stopping job: ' + err.message);
    stopBtn.disabled = false;
  }
}

// Update Job State UI
function updateJobState(job) {
  isJobRunning = job.state === 'running' || job.state === 'finalizing';

  if (job.state === 'running') {
    statusBadge.className = 'flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border bg-blue-900/30 border-blue-700/50 text-blue-300';
    statusDot.className = 'h-2 w-2 rounded-full bg-blue-400 status-running';
    statusText.textContent = `Running: ${job.run_name || ''}`;
    startBtn.classList.add('hidden');
    stopBtn.classList.remove('hidden');
    stopBtn.disabled = false;
    stopBtn.innerHTML = '<i data-lucide="square" class="w-3.5 h-3.5 fill-current"></i> Stop & Finalize';
  } else if (job.state === 'finalizing') {
    statusBadge.className = 'flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border bg-amber-900/30 border-amber-700/50 text-amber-300';
    statusDot.className = 'h-2 w-2 rounded-full bg-amber-400 status-running';
    statusText.textContent = 'Finalizing & Saving Map...';
    stopBtn.disabled = true;
    stopBtn.textContent = 'Saving...';
  } else if (job.state === 'completed' || job.state === 'stopped') {
    statusBadge.className = 'flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border bg-emerald-900/30 border-emerald-700/50 text-emerald-300';
    statusDot.className = 'h-2 w-2 rounded-full bg-emerald-400';
    statusText.textContent = job.state === 'completed' ? 'Completed' : 'Stopped & Saved';
    startBtn.classList.remove('hidden');
    startBtn.disabled = false;
    startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
    stopBtn.classList.add('hidden');
    fetchRuns();
  } else if (job.state === 'failed') {
    statusBadge.className = 'flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border bg-rose-900/30 border-rose-700/50 text-rose-300';
    statusDot.className = 'h-2 w-2 rounded-full bg-rose-500';
    statusText.textContent = 'Failed';
    startBtn.classList.remove('hidden');
    startBtn.disabled = false;
    startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
    stopBtn.classList.add('hidden');
    fetchRuns();
  } else {
    statusBadge.className = 'flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border bg-slate-800/80 border-slate-700 text-slate-300';
    statusDot.className = 'h-2 w-2 rounded-full bg-slate-500';
    statusText.textContent = 'Idle';
    startBtn.classList.remove('hidden');
    startBtn.disabled = false;
    startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
    stopBtn.classList.add('hidden');
  }

  updateMetrics(job);
  if (window.lucide) lucide.createIcons();
}

// Update Metrics Display
function updateMetrics(job) {
  if (!job) return;
  metricBagTime.textContent = (job.bag_time || 0).toFixed(1) + 's';
  metricSpeed.textContent = (job.speed || 0).toFixed(2) + 'x';
  metricScans.textContent = (job.scans || 0).toLocaleString();
  metricElapsed.textContent = (job.elapsed_sec || 0).toFixed(1) + 's';

  metricQueueOdom.textContent = job.odom_queue || 0;
  metricQueueSub.textContent = job.sub_queue || 0;
  metricQueueGlob.textContent = job.glob_queue || 0;
}

// Append Log Line
function appendLog(line) {
  const div = document.createElement('div');
  div.className = 'log-line';

  if (line.includes('[error]') || line.includes('[critical]')) {
    div.classList.add('log-err');
  } else if (line.includes('[warning]') || line.includes('[warn]')) {
    div.classList.add('log-warn');
  } else if (line.includes('[progress]')) {
    div.classList.add('log-progress');
  } else {
    div.classList.add('log-info');
  }

  div.textContent = line;
  logConsole.appendChild(div);

  if (autoScroll) {
    logConsole.scrollTop = logConsole.scrollHeight;
  }
}
