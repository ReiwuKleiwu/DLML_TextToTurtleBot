export function initTabs(buttons, panels) {
  const tabButtons = Array.from(buttons || []);
  const tabPanels = Array.from(panels || []);

  if (tabButtons.length === 0 || tabPanels.length === 0) {
    return;
  }

  function activateTab(targetId, { focusPanel = false } = {}) {
    if (!targetId) {
      return;
    }
    const targetPanel = tabPanels.find((panel) => panel.id === targetId);
    if (!targetPanel) {
      return;
    }

    tabButtons.forEach((button) => {
      const isActive = button.dataset.tabTarget === targetId;
      button.classList.toggle('active', isActive);
      button.setAttribute('aria-selected', String(isActive));
      button.setAttribute('tabindex', isActive ? '0' : '-1');
    });

    tabPanels.forEach((panel) => {
      const isActive = panel === targetPanel;
      panel.classList.toggle('active', isActive);
      panel.setAttribute('tabindex', isActive ? '0' : '-1');
      panel.hidden = !isActive;
      if (isActive) {
        panel.removeAttribute('aria-hidden');
        if (focusPanel) {
          panel.focus();
        }
        panel.querySelectorAll('.tab-panel-inner--scroll').forEach((container) => {
          container.scrollTop = 0;
        });
      } else {
        panel.setAttribute('aria-hidden', 'true');
      }
    });
  }

  function handleTabKeydown(event) {
    const { key } = event;
    if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(key)) {
      return;
    }
    event.preventDefault();
    const currentIndex = tabButtons.indexOf(event.currentTarget);
    if (currentIndex === -1) {
      return;
    }

    let nextIndex = currentIndex;
    if (key === 'ArrowLeft') {
      nextIndex = (currentIndex - 1 + tabButtons.length) % tabButtons.length;
    } else if (key === 'ArrowRight') {
      nextIndex = (currentIndex + 1) % tabButtons.length;
    } else if (key === 'Home') {
      nextIndex = 0;
    } else if (key === 'End') {
      nextIndex = tabButtons.length - 1;
    }

    const nextButton = tabButtons[nextIndex];
    if (nextButton) {
      activateTab(nextButton.dataset.tabTarget);
      nextButton.focus();
    }
  }

  tabButtons.forEach((button) => {
    button.addEventListener('click', () => {
      activateTab(button.dataset.tabTarget);
      button.focus();
    });
    button.addEventListener('keydown', handleTabKeydown);
  });

  const initialButton = tabButtons.find((button) => button.classList.contains('active')) || tabButtons[0];
  if (initialButton) {
    activateTab(initialButton.dataset.tabTarget);
  }
}
