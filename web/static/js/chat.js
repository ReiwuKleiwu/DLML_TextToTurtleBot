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
    const shouldStickToBottom = logEl.scrollHeight - logEl.scrollTop - logEl.clientHeight < 32;

    logEl.innerHTML = '';

    for (const entry of messages.slice(-120)) {
      const role = (entry && entry.role ? String(entry.role) : 'system').toLowerCase();
      const item = document.createElement('div');
      item.className = `chat-message ${role}`;

      const header = document.createElement('div');
      header.className = 'chat-message-header';

      const roleLabel = document.createElement('span');
      roleLabel.textContent = formatChatRole(role);
      header.appendChild(roleLabel);

      const tsText = formatTimestamp(entry && entry.timestamp);
      if (tsText) {
        const timeEl = document.createElement('span');
        timeEl.textContent = tsText;
        header.appendChild(timeEl);
      }

      item.appendChild(header);

      const textEl = document.createElement('div');
      textEl.className = 'chat-message-text';
      textEl.textContent = entry && typeof entry.text === 'string' ? entry.text : '';
      item.appendChild(textEl);

      logEl.appendChild(item);
    }

    if (shouldStickToBottom) {
      logEl.scrollTop = logEl.scrollHeight;
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
