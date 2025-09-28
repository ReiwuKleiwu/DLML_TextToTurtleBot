export function formatNumber(value) {
  return Number.isFinite(value) ? value.toFixed(2) : '—';
}

export function formatChatRole(role) {
  if (!role) {
    return 'Message';
  }
  const normalized = role.toLowerCase();
  switch (normalized) {
    case 'user':
      return 'User';
    case 'assistant':
      return 'LLM';
    case 'system':
      return 'System';
    default:
      return normalized.charAt(0).toUpperCase() + normalized.slice(1);
  }
}

export function formatTimestamp(value) {
  if (typeof value !== 'number') {
    return '';
  }
  const date = new Date(value * 1000);
  if (Number.isNaN(date.getTime())) {
    return '';
  }
  const now = Date.now();
  const diff = Math.abs(now - date.getTime());
  if (diff < 86_400_000) {
    return date.toLocaleTimeString();
  }
  return `${date.toLocaleDateString()} ${date.toLocaleTimeString()}`;
}
