import { formatChatRole, formatTimestamp } from './formatters.js';

export function createChatController({
  logEl,
  formEl,
  inputEl,
  statusEl,
  sendButton,
}) {
  let isSending = false;
  let statusTimer = null;

  function render(chat) {
    if (!logEl) {
      return;
    }

    const messages = Array.isArray(chat) ? chat : [];
    const trimmed = messages.slice(-120);
    const shouldStickToBottom = logEl.scrollHeight - logEl.scrollTop - logEl.clientHeight < 32;
    const existingNodes = new Map();
    Array.from(logEl.children).forEach((child) => {
      if (child.dataset && child.dataset.key) {
        existingNodes.set(child.dataset.key, child);
      }
    });

    const desiredNodes = [];

    trimmed.forEach((entry, index) => {
      const key = createMessageKey(entry, index);
      let node = existingNodes.get(key);
      if (node) {
        existingNodes.delete(key);
      } else {
        node = buildMessageNode();
        node.dataset.key = key;
      }
      updateMessageNode(node, entry);
      desiredNodes.push(node);
    });

    existingNodes.forEach((node) => {
      if (node.parentNode === logEl) {
        logEl.removeChild(node);
      }
    });

    desiredNodes.forEach((node, index) => {
      const current = logEl.children[index];
      if (current !== node) {
        logEl.insertBefore(node, current || null);
      }
    });

    if (shouldStickToBottom) {
      logEl.scrollTop = logEl.scrollHeight;
    }
  }

  function createMessageKey(entry, index) {
    const role = (entry && entry.role ? String(entry.role) : 'system').toLowerCase();
    const timestamp = entry && typeof entry.timestamp === 'number' ? entry.timestamp : null;
    if (timestamp !== null) {
      return `ts:${timestamp}:${role}`;
    }
    const text = entry && typeof entry.text === 'string' ? entry.text : '';
    const textSnippet = text.length > 32 ? text.slice(0, 32) : text;
    return `idx:${index}:${role}:${textSnippet}`;
  }

  function buildMessageNode() {
    const item = document.createElement('div');
    const header = document.createElement('div');
    header.className = 'chat-message-header';

    const roleLabel = document.createElement('span');
    header.appendChild(roleLabel);

    const timeEl = document.createElement('span');
    header.appendChild(timeEl);

    item.appendChild(header);

    const textEl = document.createElement('div');
    textEl.className = 'chat-message-text';
    item.appendChild(textEl);

    return item;
  }

  function updateMessageNode(node, entry) {
    const role = (entry && entry.role ? String(entry.role) : 'system').toLowerCase();
    node.className = `chat-message ${role}`;

    const metadata = entry && typeof entry.metadata === 'object' ? entry.metadata : null;

    const header = node.querySelector('.chat-message-header');
    if (header) {
      const [roleLabel, timeEl] = header.children;
      if (roleLabel) {
        roleLabel.textContent = formatChatRole(role);
      }
      if (timeEl) {
        const tsText = formatTimestamp(entry && entry.timestamp);
        if (tsText) {
          timeEl.textContent = tsText;
          timeEl.style.display = '';
        } else {
          timeEl.textContent = '';
          timeEl.style.display = 'none';
        }
      }
    }

    const textEl = node.querySelector('.chat-message-text');
    if (textEl) {
      textEl.textContent = entry && typeof entry.text === 'string' ? entry.text : '';
    }

    const audioMeta = metadata && metadata.audio && typeof metadata.audio === 'object'
      ? metadata.audio
      : null;

    let audioWrapper = node.querySelector('.chat-message-audio');
    let audioEl = audioWrapper ? audioWrapper.querySelector('audio') : null;

    if (audioMeta && audioMeta.url) {
      if (!audioWrapper) {
        audioWrapper = document.createElement('div');
        audioWrapper.className = 'chat-message-audio';
        audioEl = document.createElement('audio');
        audioEl.controls = true;
        audioEl.preload = 'none';
        audioWrapper.appendChild(audioEl);
        node.appendChild(audioWrapper);
      }
      if (audioEl) {
        const targetSrc = String(audioMeta.url);
        if (audioEl.dataset.src !== targetSrc) {
          const wasPlaying = !audioEl.paused && !audioEl.ended;
          audioEl.src = targetSrc;
          audioEl.dataset.src = targetSrc;
          if (wasPlaying) {
            audioEl.play().catch(() => {
              /* ignore autoplay prevention */
            });
          }
        }
        audioEl.title = audioMeta.provider
          ? `TTS playback (${audioMeta.provider})`
          : 'TTS playback';
        if (audioMeta.format) {
          audioEl.type = `audio/${audioMeta.format}`;
          audioEl.dataset.format = audioMeta.format;
        } else {
          audioEl.removeAttribute('type');
          delete audioEl.dataset.format;
        }
      }
    } else if (audioWrapper) {
      // Preserve existing playback if already attached; otherwise remove stale wrapper.
      if (!audioEl || audioEl.paused || audioEl.ended) {
        node.removeChild(audioWrapper);
      }
    }
  }

  function setStatus(message, { error = false, autoClear = true } = {}) {
    if (!statusEl) {
      return;
    }
    window.clearTimeout(statusTimer);
    if (!message) {
      statusEl.textContent = '';
      statusEl.classList.remove('error');
      return;
    }
    statusEl.textContent = message;
    statusEl.classList.toggle('error', Boolean(error));
    if (!error && autoClear) {
      statusTimer = window.setTimeout(() => setStatus(''), 2500);
    }
  }

  function setSending(flag) {
    if (sendButton) {
      sendButton.disabled = flag;
    }
    if (inputEl) {
      inputEl.classList.toggle('sending', flag);
    }
  }

  async function sendMessage(message) {
    const response = await fetch('/api/chat', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ message }),
    });

    if (!response.ok) {
      let detail;
      try {
        const payload = await response.json();
        detail = payload && (payload.detail || payload.message);
      } catch (error) {
        detail = null;
      }
      throw new Error(detail || `Chat send failed (HTTP ${response.status})`);
    }

    return response.json().catch(() => ({}));
  }

  async function handleSubmit(event) {
    event.preventDefault();
    if (!inputEl || isSending) {
      return;
    }

    const message = inputEl.value.trim();
    if (!message) {
      setStatus('Enter a message to send', { error: true, autoClear: false });
      return;
    }

    try {
      isSending = true;
      setSending(true);
      setStatus('Sending...', { autoClear: false });
      await sendMessage(message);
      inputEl.value = '';
      setStatus('Message queued');
      inputEl.focus();
    } catch (error) {
      setStatus(error.message || 'Failed to send message', { error: true, autoClear: false });
    } finally {
      isSending = false;
      setSending(false);
    }
  }

  function handleKeyDown(event) {
    if (event.key === 'Enter' && !event.shiftKey) {
      event.preventDefault();
      if (formEl && typeof formEl.requestSubmit === 'function') {
        formEl.requestSubmit();
      } else if (formEl) {
        formEl.dispatchEvent(new Event('submit', { cancelable: true, bubbles: true }));
      }
    }
  }

  function handleInput() {
    setStatus('');
  }

  if (formEl) {
    formEl.addEventListener('submit', handleSubmit);
  }
  if (inputEl) {
    inputEl.addEventListener('keydown', handleKeyDown);
    inputEl.addEventListener('input', handleInput);
  }

  return {
    render,
  };
}
