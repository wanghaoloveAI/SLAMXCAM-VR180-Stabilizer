(function () {
  'use strict';

  const $ = function (selector) { return document.querySelector(selector); };
  const video = $('#previewVideo');
  const canvas = $('#scope');
  let objectUrl = null;
  let jobTimer = null;

  function savedTheme() {
    try { return window.localStorage.getItem('slam-xcam-ui-theme'); }
    catch (error) { return null; }
  }

  function applyTheme(theme, persist) {
    const selected = theme === 'pixel' ? 'pixel' : 'apple';
    document.body.classList.remove('apple', 'pixel');
    document.body.classList.add(selected);
    document.querySelectorAll('.theme-option').forEach(function (option) {
      const active = option.dataset.theme === selected;
      option.classList.toggle('active', active);
      option.setAttribute('aria-checked', active ? 'true' : 'false');
    });
    $('#appStatus').textContent = selected === 'pixel' ? 'SYSTEM READY / PIXEL UI' : 'Ready';
    if (persist) {
      try { window.localStorage.setItem('slam-xcam-ui-theme', selected); }
      catch (error) { /* Local file privacy settings may disable storage. */ }
    }
    window.requestAnimationFrame(drawScope);
  }

  function closeThemeMenu() {
    $('#uiThemeMenu').classList.remove('open');
    $('#uiMenuTrigger').setAttribute('aria-expanded', 'false');
  }

  const requestedTheme = new URLSearchParams(window.location.search).get('theme');
  applyTheme(requestedTheme === 'pixel' || requestedTheme === 'apple' ? requestedTheme : (savedTheme() || 'apple'), false);
  $('#uiMenuTrigger').addEventListener('click', function (event) {
    event.stopPropagation();
    const open = $('#uiThemeMenu').classList.toggle('open');
    this.setAttribute('aria-expanded', open ? 'true' : 'false');
  });
  document.querySelectorAll('.theme-option').forEach(function (option) {
    option.addEventListener('click', function () { applyTheme(option.dataset.theme, true); closeThemeMenu(); });
  });
  document.addEventListener('click', function (event) {
    if (!event.target.closest('.menu-group')) closeThemeMenu();
  });
  document.addEventListener('keydown', function (event) { if (event.key === 'Escape') closeThemeMenu(); });

  function drawScope() {
    const rect = canvas.getBoundingClientRect();
    const scale = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.floor(rect.width * scale));
    canvas.height = Math.max(1, Math.floor(rect.height * scale));
    const ctx = canvas.getContext('2d');
    ctx.scale(scale, scale);
    const w = rect.width;
    const h = rect.height;
    const isPixel = document.body.classList.contains('pixel');
    ctx.fillStyle = isPixel ? '#101010' : '#151515';
    ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = isPixel ? '#303030' : '#333333';
    ctx.lineWidth = 1;
    for (let x = 0; x < w; x += 28) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    }
    for (let y = 0; y < h; y += 28) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }
    const radius = Math.min(h * 0.34, w * 0.2);
    [w * 0.28, w * 0.72].forEach(function (cx, index) {
      ctx.fillStyle = isPixel ? '#252525' : '#242424';
      ctx.beginPath(); ctx.arc(cx, h * 0.52, radius, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = index ? (isPixel ? '#777777' : '#bcbcbc') : (isPixel ? '#a8a8a8' : '#f2f2f2');
      ctx.lineWidth = isPixel ? 4 : 2;
      ctx.stroke();
      ctx.strokeStyle = isPixel ? '#555555' : '#686868';
      ctx.lineWidth = 1;
      for (let ring = 1; ring < 4; ring += 1) {
        ctx.beginPath(); ctx.arc(cx, h * 0.52, radius * ring / 4, 0, Math.PI * 2); ctx.stroke();
      }
      ctx.beginPath();
      ctx.moveTo(cx - radius, h * 0.52); ctx.lineTo(cx + radius, h * 0.52);
      ctx.moveTo(cx, h * 0.52 - radius); ctx.lineTo(cx, h * 0.52 + radius); ctx.stroke();
      ctx.fillStyle = '#f5f5f5';
      ctx.font = isPixel ? 'bold 14px Consolas' : '600 13px Segoe UI';
      ctx.fillText(index ? 'RIGHT' : 'LEFT', cx - 22, h * 0.52 + 5);
    });
  }

  function appendLog(message) {
    const now = new Date().toLocaleTimeString('en-GB', { hour12: false });
    $('#log').textContent += String.fromCharCode(10) + '[' + now + '] ' + message;
    $('#log').scrollTop = $('#log').scrollHeight;
  }

  function updateMeta() {
    $('#previewMeta').textContent = 'SLAM XCAM ' + $('#model').value + ' / ' + $('#videoMode').value + ' / 6D VQF + Reference Renderer';
  }

  function selectPage(pageId, updateHash) {
    if (!document.getElementById(pageId)) pageId = 'stabilizerPage';
    document.querySelectorAll('.page-tab').forEach(function (item) { item.classList.toggle('active', item.dataset.page === pageId); });
    document.querySelectorAll('.app-page').forEach(function (page) { page.classList.toggle('active', page.id === pageId); });
    if (updateHash) window.history.replaceState(null, '', '#' + pageId);
    if (pageId === 'stabilizerPage') window.requestAnimationFrame(drawScope);
  }

  document.querySelectorAll('.page-tab').forEach(function (tab) {
    tab.addEventListener('click', function () { selectPage(tab.dataset.page, true); });
  });
  selectPage(window.location.hash.slice(1) || 'stabilizerPage', false);
  window.requestAnimationFrame(function () { window.scrollTo(0, 0); });
  window.addEventListener('load', function () { window.setTimeout(function () { window.scrollTo(0, 0); }, 0); });

  document.querySelectorAll('.tab').forEach(function (tab) {
    tab.addEventListener('click', function () {
      document.querySelectorAll('.tab').forEach(function (item) { item.classList.toggle('active', item === tab); });
      document.querySelectorAll('.tab-panel').forEach(function (panel) { panel.classList.toggle('active', panel.id === tab.dataset.tab); });
    });
  });

  $('#chooseVideo').addEventListener('click', function () { $('#videoInput').click(); });
  $('#chooseImu').addEventListener('click', function () { $('#imuInput').click(); });
  $('#videoInput').addEventListener('change', function (event) {
    const file = event.target.files[0];
    if (!file) return;
    if (objectUrl) URL.revokeObjectURL(objectUrl);
    objectUrl = URL.createObjectURL(file);
    video.src = objectUrl;
    video.style.display = 'block';
    canvas.style.display = 'none';
    $('#videoName').value = file.name;
    $('#previewTitle').textContent = file.name;
    $('#sourceState').textContent = document.body.classList.contains('pixel') ? 'SOURCE ONLINE' : 'Video ready';
    $('#folderMeta').innerHTML = 'Video folder: local browser selection<br>Save folder: source video folder';
    const base = file.name.replace(/\.[^.]+$/, '');
    const candidate = base + '_motion.slamimu';
    $('#imuName').value = candidate;
    $('#matchState').textContent = 'Auto match candidate: ' + candidate;
    appendLog('Video loaded: ' + file.name);
    appendLog('IMU auto-match candidate prepared');
  });
  $('#imuInput').addEventListener('change', function (event) {
    const file = event.target.files[0];
    if (!file) return;
    $('#imuName').value = file.name;
    $('#matchState').textContent = 'Manual IMU: ' + file.name;
    appendLog('IMU loaded: ' + file.name);
  });

  $('#cameraRefresh').addEventListener('click', function () {
    const button = this;
    const original = button.textContent;
    button.disabled = true;
    button.textContent = document.body.classList.contains('pixel') ? 'SCANNING...' : '正在刷新...';
    $('#cameraStatus').textContent = document.body.classList.contains('pixel') ? 'SCANNING' : 'Scanning';
    window.setTimeout(function () {
      button.disabled = false;
      button.textContent = original;
      $('#cameraStatus').textContent = document.body.classList.contains('pixel') ? 'READY' : 'Ready';
    }, 700);
  });

  $('#cameraConnect').addEventListener('click', function () {
    const disconnected = this.dataset.disconnected === 'true';
    this.dataset.disconnected = disconnected ? 'false' : 'true';
    if (document.body.classList.contains('pixel')) {
      this.textContent = disconnected ? 'DISCONNECT' : 'CONNECT';
      $('#cameraStatus').textContent = disconnected ? 'READY' : 'OFFLINE';
    } else {
      this.textContent = disconnected ? '断开连接' : '重新连接';
      $('#cameraStatus').textContent = disconnected ? 'Ready' : 'Offline';
    }
  });

  function showDeviceFiles(showFiles) {
    $('#cameraInfoView').hidden = showFiles;
    $('#deviceFilesView').hidden = !showFiles;
    $('#deviceFilesButton').classList.toggle('active', showFiles);
  }

  function updateDeviceFileSelection() {
    const checks = Array.from(document.querySelectorAll('.device-file input[type="checkbox"]'));
    const selected = checks.filter(function (check) { return check.checked; });
    checks.forEach(function (check) { check.closest('.device-file').classList.toggle('selected', check.checked); });
    $('#selectedFileCount').textContent = document.body.classList.contains('pixel') ? selected.length + ' SELECTED' : '已选择 ' + selected.length + ' 个';
    $('#exportDeviceFiles').disabled = selected.length === 0;
    const visibleChecks = checks.filter(function (check) { return !check.closest('.device-file').hidden; });
    $('#selectAllDeviceFiles').checked = visibleChecks.length > 0 && visibleChecks.every(function (check) { return check.checked; });
    $('#selectAllDeviceFiles').indeterminate = visibleChecks.some(function (check) { return check.checked; }) && !$('#selectAllDeviceFiles').checked;
  }

  $('#deviceInfoButton').addEventListener('click', function () { showDeviceFiles(false); });
  $('#deviceFilesButton').addEventListener('click', function () {
    const button = this;
    button.disabled = true;
    $('#appStatus').textContent = document.body.classList.contains('pixel') ? 'READING DEVICE FILES...' : '正在读取设备文件...';
    window.setTimeout(function () {
      button.disabled = false;
      showDeviceFiles(true);
      $('#appStatus').textContent = document.body.classList.contains('pixel') ? '24 FILES READY' : '已读取设备文件';
    }, 450);
  });

  document.querySelectorAll('.device-file input[type="checkbox"]').forEach(function (check) {
    check.addEventListener('change', updateDeviceFileSelection);
  });
  $('#selectAllDeviceFiles').addEventListener('change', function () {
    const checked = this.checked;
    document.querySelectorAll('.device-file').forEach(function (file) {
      if (!file.hidden) file.querySelector('input[type="checkbox"]').checked = checked;
    });
    updateDeviceFileSelection();
  });
  document.querySelectorAll('.filter-btn').forEach(function (button) {
    button.addEventListener('click', function () {
      document.querySelectorAll('.filter-btn').forEach(function (item) { item.classList.toggle('active', item === button); });
      let visibleCount = 0;
      document.querySelectorAll('.device-file').forEach(function (file) {
        file.hidden = button.dataset.filter !== 'all' && file.dataset.kind !== button.dataset.filter;
        if (!file.hidden) visibleCount += 1;
      });
      $('#fileEmptyState').classList.toggle('visible', visibleCount === 0);
      updateDeviceFileSelection();
    });
  });
  $('#refreshDeviceFiles').addEventListener('click', function () {
    const button = this;
    const original = button.textContent;
    button.disabled = true;
    button.textContent = document.body.classList.contains('pixel') ? 'READING...' : '读取中...';
    window.setTimeout(function () { button.disabled = false; button.textContent = original; }, 650);
  });
  $('#exportDeviceFiles').addEventListener('click', function () {
    const selected = document.querySelectorAll('.device-file input:checked').length;
    if (!selected) return;
    const button = this;
    button.disabled = true;
    button.textContent = document.body.classList.contains('pixel') ? 'EXPORTING ' + selected + ' FILES...' : '正在导出 ' + selected + ' 个文件...';
    window.setTimeout(function () {
      button.textContent = document.body.classList.contains('pixel') ? 'EXPORT SELECTED' : '导出所选文件';
      updateDeviceFileSelection();
      $('#appStatus').textContent = document.body.classList.contains('pixel') ? 'EXPORT COMPLETE' : '导出完成';
    }, 1100);
  });
  updateDeviceFileSelection();
  if (new URLSearchParams(window.location.search).get('deviceFiles') === '1') showDeviceFiles(true);

  $('#editChoose').addEventListener('click', function () { $('#editVideoInput').click(); });
  $('#editVideoInput').addEventListener('change', function (event) {
    const file = event.target.files[0];
    if (!file) return;
    $('#editClipName').textContent = file.name;
  });
  $('#editorExport').addEventListener('click', function () {
    const button = this;
    const original = button.textContent;
    button.disabled = true;
    button.textContent = document.body.classList.contains('pixel') ? 'EXPORT QUEUED' : '已加入导出队列';
    window.setTimeout(function () { button.disabled = false; button.textContent = original; }, 1200);
  });

  [
    ['exposure', 'exposureValue', function (value) { return (value / 10).toFixed(1) + ' EV'; }],
    ['contrast', 'contrastValue', function (value) { return value; }],
    ['saturation', 'saturationValue', function (value) { return value; }],
    ['temperature', 'temperatureValue', function (value) { return value; }]
  ].forEach(function (setting) {
    $('#' + setting[0]).addEventListener('input', function (event) { $('#' + setting[1]).textContent = setting[2](event.target.value); });
  });
  $('#model').addEventListener('change', updateMeta);
  $('#videoMode').addEventListener('change', updateMeta);

  $('#play').addEventListener('click', function () {
    if (!video.src) { appendLog('Preview requires a local video'); return; }
    if (video.paused) video.play(); else video.pause();
  });
  video.addEventListener('play', function () { $('#play').textContent = 'II'; });
  video.addEventListener('pause', function () { $('#play').textContent = '▶'; });
  video.addEventListener('timeupdate', function () {
    if (!video.duration) return;
    $('#timeline').value = video.currentTime / video.duration * 1000;
    const format = function (seconds) { return String(Math.floor(seconds / 60)).padStart(2, '0') + ':' + String(Math.floor(seconds % 60)).padStart(2, '0'); };
    $('#timecode').textContent = format(video.currentTime) + ' / ' + format(video.duration);
  });
  $('#timeline').addEventListener('input', function (event) { if (video.duration) video.currentTime = video.duration * event.target.value / 1000; });

  $('#start').addEventListener('click', function () {
    if (jobTimer) return;
    let progress = 0;
    const started = Date.now();
    $('#start').disabled = true;
    $('#start').textContent = 'Processing...';
    appendLog('Pipeline started: 6D VQF -> horizon correction -> reprojection');
    jobTimer = window.setInterval(function () {
      progress = Math.min(100, progress + 2);
      const elapsed = Math.floor((Date.now() - started) / 1000);
      const eta = progress ? Math.ceil(elapsed * (100 - progress) / progress) : 0;
      $('#barFill').style.width = progress + '%';
      $('#progressPercent').textContent = progress + '%';
      $('#progressText').textContent = progress < 100 ? 'Rendering frames' : 'Complete';
      $('#timeMeta').textContent = 'Elapsed 00:' + String(elapsed).padStart(2, '0') + ' / ETA 00:' + String(eta).padStart(2, '0');
      if (progress % 20 === 0 && progress < 100) appendLog('Rendered ' + progress + '% / GPU backend simulation');
      if (progress === 100) {
        window.clearInterval(jobTimer);
        jobTimer = null;
        $('#start').disabled = false;
        $('#start').textContent = 'Start Stabilization';
        appendLog('Output complete: source_folder/*_stabilized.mp4');
      }
    }, 90);
  });

  window.addEventListener('resize', drawScope);
  drawScope();
}());
