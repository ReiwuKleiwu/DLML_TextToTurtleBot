import { formatNumber, formatTimestamp } from './formatters.js';

export function updateStatus({ behaviourEl, navigationEl, targetEl, timeEl }, state) {
  const paused = Boolean(state.behaviour_tree_paused);
  if (behaviourEl) {
    behaviourEl.textContent = paused ? 'Tree Paused' : 'Tree Active';
    behaviourEl.classList.toggle('paused', paused);
    behaviourEl.classList.toggle('active', !paused);
  }

  const navStatus = state.navigation && state.navigation.status ? state.navigation.status : 'idle';
  if (navigationEl) {
    navigationEl.textContent = `Nav: ${navStatus}`;
    navigationEl.classList.toggle('active', Boolean(state.navigation && state.navigation.status));
  }

  if (targetEl) {
    const target = state.target;
    if (target && target.class_name) {
      targetEl.textContent = `Target: ${target.class_name}`;
      targetEl.classList.add('target');
      targetEl.classList.add('active');
    } else {
      targetEl.textContent = 'Target: None';
      targetEl.classList.remove('target');
      targetEl.classList.remove('active');
    }
  }

  if (timeEl) {
    const timestamp = typeof state.timestamp === 'number'
      ? new Date(state.timestamp * 1000)
      : new Date();
    timeEl.textContent = `Updated ${timestamp.toLocaleTimeString()}`;
    timeEl.classList.remove('status-error');
  }
}

export function updateActiveCommand(containerEl, commands) {
  if (!containerEl) {
    return;
  }
  const active = commands && commands.active_command;
  containerEl.innerHTML = '';
  if (!active) {
    containerEl.appendChild(createEmptyState('None'));
    return;
  }
  containerEl.appendChild(createCommandEntry(active, 'active'));
}

export function updatePendingCommands(listEl, commands) {
  if (!listEl) {
    return;
  }
  const pending = (commands && Array.isArray(commands.pending_commands)) ? commands.pending_commands : [];
  listEl.innerHTML = '';

  if (pending.length === 0) {
    const noneItem = document.createElement('li');
    noneItem.className = 'command-list-empty';
    noneItem.appendChild(createEmptyState('Queue empty'));
    listEl.appendChild(noneItem);
    return;
  }

  const display = pending.slice(0, 6);

  display.forEach((command, index) => {
    const item = document.createElement('li');
    item.className = 'command-list-item';
    const variant = index === 0 ? 'pending-front' : 'pending';
    item.appendChild(createCommandEntry(command, variant));
    listEl.appendChild(item);
  });
}

export function updateTargetDetails(containerEl, target) {
  if (!containerEl) {
    return;
  }
  if (!target || !target.class_name) {
    containerEl.textContent = 'No target selected.';
    return;
  }

  const parts = [`Class: ${target.class_name}`];
  if (typeof target.confidence === 'number') {
    parts.push(`Confidence: ${(target.confidence * 100).toFixed(1)}%`);
  }
  if (typeof target.distance_to_robot === 'number') {
    parts.push(`Distance: ${target.distance_to_robot.toFixed(2)} m`);
  }
  if (target.world) {
    parts.push(`World: (${formatNumber(target.world.x)}, ${formatNumber(target.world.y)})`);
  }
  containerEl.textContent = parts.join('\n');
}

export function updateNavigationDetails(containerEl, navigation) {
  if (!containerEl) {
    return;
  }
  if (!navigation || !navigation.goal || !navigation.goal.position) {
    containerEl.textContent = 'No navigation goal.';
    return;
  }

  const status = navigation.status || 'pending';
  const goal = navigation.goal;
  const parts = [`Status: ${status}`];
  const pos = goal.position;
  parts.push(`Goal: (${formatNumber(pos.x)}, ${formatNumber(pos.y)})`);
  if (typeof goal.yaw === 'number') {
    parts.push(`Yaw: ${goal.yaw.toFixed(2)} rad`);
  }
  if (goal.frame_id) {
    parts.push(`Frame: ${goal.frame_id}`);
  }
  containerEl.textContent = parts.join('\n');
}

export function updatePersistentObjects(listEl, mapData) {
  if (!listEl) {
    return;
  }
  const objects = (mapData && Array.isArray(mapData.objects)) ? mapData.objects : [];
  listEl.innerHTML = '';

  if (objects.length === 0) {
    const noneEl = document.createElement('li');
    noneEl.className = 'object-list-empty';
    noneEl.appendChild(createEmptyState('No persistent objects.'));
    listEl.appendChild(noneEl);
    return;
  }

  for (const object of objects.slice(0, 8)) {
    const item = document.createElement('li');
    item.className = 'object-list-item';
    item.appendChild(createObjectEntry(object));
    listEl.appendChild(item);
  }
}

export function showError(timeEl, error) {
  if (!timeEl) {
    return;
  }
  timeEl.textContent = `Error: ${error.message || error}`;
  timeEl.classList.add('status-error');
}

export function clearError(timeEl) {
  if (!timeEl) {
    return;
  }
  timeEl.classList.remove('status-error');
}

function createCommandEntry(command, variant) {
  const wrapper = document.createElement('div');
  wrapper.className = `command-entry command-entry--${variant}`;

  const header = document.createElement('div');
  header.className = 'command-entry__header';

  const title = document.createElement('span');
  title.className = 'command-entry__title';
  title.textContent = command.command_type || 'command';
  header.appendChild(title);

  if (command.command_id) {
    const badge = document.createElement('span');
    badge.className = 'command-entry__badge';
    badge.textContent = command.command_id;
    header.appendChild(badge);
  }

  wrapper.appendChild(header);

  const meta = document.createElement('div');
  meta.className = 'command-entry__meta';

  if (command.parameters && Object.keys(command.parameters).length > 0) {
    meta.appendChild(createMetaRow('Params', JSON.stringify(command.parameters)));
  }

  if (command.timestamp) {
    const timestampText = formatTimestamp(command.timestamp);
    if (timestampText) {
      meta.appendChild(createMetaRow('Queued', timestampText));
    }
  }

  if (meta.childElementCount > 0) {
    wrapper.appendChild(meta);
  }

  return wrapper;
}

function createMetaRow(label, value) {
  const row = document.createElement('div');
  row.className = 'command-entry__meta-row';

  const labelEl = document.createElement('span');
  labelEl.className = 'command-entry__meta-label';
  labelEl.textContent = label;

  const valueEl = document.createElement('span');
  valueEl.className = 'command-entry__meta-value';
  valueEl.textContent = value;

  row.appendChild(labelEl);
  row.appendChild(valueEl);
  return row;
}

function createEmptyState(message) {
  const wrapper = document.createElement('div');
  wrapper.className = 'empty-state';
  wrapper.textContent = message;
  return wrapper;
}

function createObjectEntry(object) {
  const wrapper = document.createElement('div');
  wrapper.className = 'object-entry';
  if (object && object.is_target) {
    wrapper.classList.add('object-entry--target');
  }

  const header = document.createElement('div');
  header.className = 'object-entry__header';

  const title = document.createElement('span');
  title.className = 'object-entry__title';
  title.textContent = object && object.class_name ? object.class_name : 'Object';
  header.appendChild(title);

  if (object && typeof object.total_detections === 'number') {
    const badge = document.createElement('span');
    badge.className = 'object-entry__badge';
    badge.textContent = `${object.total_detections} detections`;
    header.appendChild(badge);
  }

  wrapper.appendChild(header);

  const meta = document.createElement('div');
  meta.className = 'object-entry__meta';

  if (object && object.world) {
    meta.appendChild(createObjectMetaRow('World', `(${formatNumber(object.world.x)}, ${formatNumber(object.world.y)})`));
  }

  if (object && typeof object.last_seen === 'number') {
    const seenDate = new Date(object.last_seen * 1000);
    if (!Number.isNaN(seenDate.getTime())) {
      meta.appendChild(createObjectMetaRow('Last Seen', seenDate.toLocaleTimeString()));
    }
  }

  if (object && typeof object.confidence === 'number') {
    meta.appendChild(createObjectMetaRow('Confidence', `${(object.confidence * 100).toFixed(1)}%`));
  }

  if (meta.childElementCount > 0) {
    wrapper.appendChild(meta);
  }

  return wrapper;
}

function createObjectMetaRow(label, value) {
  const row = document.createElement('div');
  row.className = 'object-entry__meta-row';

  const labelEl = document.createElement('span');
  labelEl.className = 'object-entry__meta-label';
  labelEl.textContent = label;

  const valueEl = document.createElement('span');
  valueEl.className = 'object-entry__meta-value';
  valueEl.textContent = value;

  row.appendChild(labelEl);
  row.appendChild(valueEl);
  return row;
}
