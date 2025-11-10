import { createChatController } from './chat.js';
import { createMapController } from './map.js';
import { initTabs } from './tabs.js';
import {
  updateStatus,
  updateActiveCommand,
  updatePendingCommands,
  updateTargetDetails,
  updateNavigationDetails,
  updatePersistentObjects,
  showError,
  clearError,
} from './ui-updates.js';

const REFRESH_INTERVAL_MS = 250;

const elements = {
  canvas: document.getElementById('mission-map'),
  status: {
    behaviourEl: document.getElementById('status-behaviour'),
    navigationEl: document.getElementById('status-navigation'),
    targetEl: document.getElementById('status-target'),
    timeEl: document.getElementById('status-time'),
  },
  statusCursorEl: document.getElementById('status-cursor'),
  resetViewButton: document.getElementById('reset-view'),
  activeCommandEl: document.getElementById('active-command'),
  pendingCommandsEl: document.getElementById('pending-commands'),
  targetDetailsEl: document.getElementById('target-details'),
  navigationDetailsEl: document.getElementById('navigation-details'),
  persistentObjectsEl: document.getElementById('persistent-objects'),
  chat: {
    logEl: document.getElementById('chat-log'),
    formEl: document.getElementById('chat-form'),
    inputEl: document.getElementById('chat-input'),
    statusEl: document.getElementById('chat-status'),
    sendButton: document.getElementById('chat-send'),
  },
  tabs: {
    buttons: document.querySelectorAll('[data-tab-target]'),
    panels: document.querySelectorAll('[data-tab-panel]'),
  },
};

const chatController = createChatController(elements.chat);
const mapController = createMapController({
  canvas: elements.canvas,
  statusCursorEl: elements.statusCursorEl,
  resetButton: elements.resetViewButton,
});

initTabs(elements.tabs.buttons, elements.tabs.panels);

function updateUI(state) {
  updateStatus(elements.status, state);
  updateActiveCommand(elements.activeCommandEl, state.commands);
  updatePendingCommands(elements.pendingCommandsEl, state.commands);
  updateTargetDetails(elements.targetDetailsEl, state.target);
  updateNavigationDetails(elements.navigationDetailsEl, state.navigation);
  updatePersistentObjects(elements.persistentObjectsEl, state.persistent_map);
  chatController.render(state.chat);
  mapController.render(state);
}

function poll() {
  fetch('/api/state', { cache: 'no-store' })
    .then((response) => {
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      return response.json();
    })
    .then((state) => {
      updateUI(state);
      clearError(elements.status.timeEl);
    })
    .catch((error) => {
      showError(elements.status.timeEl, error);
    })
    .finally(() => {
      window.setTimeout(poll, REFRESH_INTERVAL_MS);
    });
}

poll();
