import { toggleHidden, clamp } from '../utils/dom.js';

const EDGE_PADDING = 16;
const MIN_PANEL_WIDTH = 320;
const MAX_PANEL_WIDTH = 720;

export class OverlayPanel {
  constructor(container) {
    this.container = container || document.body;
    this.onAction = null;
    this.overlays = new Map();
    this.spawnIndex = 0;
    this.zIndexSeed = 30;
    this.dragState = null;
    this.handlePointerMove = this.handlePointerMove.bind(this);
    this.handlePointerUp = this.handlePointerUp.bind(this);
    this.handleWindowResize = this.handleWindowResize.bind(this);
    window.addEventListener('resize', this.handleWindowResize);
  }

  setActionHandler(handler) {
    this.onAction = handler;
  }

  show(content, options = {}) {
    const id = options.id || this.generateId();
    const overlay = this.ensureOverlay(id, options);
    this.renderContent(overlay, content);
    toggleHidden(overlay.root, false);
    this.bringToFront(overlay);
    this.syncSize(overlay);
    this.ensureWithinBounds(overlay);
    window.requestAnimationFrame(() => {
      this.syncSize(overlay);
      this.ensureWithinBounds(overlay);
    });
    return id;
  }

  hide(options = {}) {
    if (options?.id) {
      const overlay = this.overlays.get(options.id);
      if (!overlay) {
        return;
      }
      overlay.root.remove();
      this.overlays.delete(options.id);
      if (this.dragState?.overlayId === options.id) {
        this.endDrag();
      }
      return;
    }
    this.overlays.forEach(entry => entry.root.remove());
    this.overlays.clear();
    this.endDrag();
  }

  isOpen(id) {
    if (!id) {
      return this.overlays.size > 0;
    }
    return this.overlays.has(id);
  }

  ensureOverlay(id, options = {}) {
    let overlay = this.overlays.get(id);
    if (!overlay) {
      overlay = this.createOverlay(id, options);
      this.overlays.set(id, overlay);
    } else if (options.position) {
      this.applyPosition(overlay, options.position);
    }
    return overlay;
  }

  createOverlay(id, options) {
    const root = document.createElement('div');
    root.className = 'overlay-panel overlay-panel--floating hidden';
    root.dataset.overlayId = id;
    root.tabIndex = 0;
    root.setAttribute('role', 'dialog');

    const header = document.createElement('div');
    header.className = 'overlay-panel__header';

    const title = document.createElement('div');
    title.className = 'overlay-panel__title';
    header.appendChild(title);

    const controls = document.createElement('div');
    controls.className = 'overlay-panel__controls';

    const closeBtn = document.createElement('button');
    closeBtn.type = 'button';
    closeBtn.className = 'overlay-panel__close';
    closeBtn.setAttribute('aria-label', 'Close overlay');
    closeBtn.textContent = '×';
    controls.appendChild(closeBtn);
    header.appendChild(controls);

    const subtitle = document.createElement('div');
    subtitle.className = 'overlay-panel__subtitle';
    subtitle.hidden = true;

    const body = document.createElement('div');
    body.className = 'overlay-panel__body';

    root.appendChild(header);
    root.appendChild(subtitle);
    root.appendChild(body);

    this.container.appendChild(root);

    const overlay = {
      id,
      root,
      header,
      titleEl: title,
      subtitleEl: subtitle,
      bodyEl: body,
    };

    closeBtn.addEventListener('click', () => this.hide({ id }));
    header.addEventListener('pointerdown', event => this.startDrag(event, overlay));
    root.addEventListener('pointerdown', () => this.bringToFront(overlay));
    root.addEventListener('keydown', event => {
      if (event.key === 'Escape') {
        event.stopPropagation();
        this.hide({ id });
      }
    });

    this.applyInitialPosition(overlay, options);
    this.bringToFront(overlay);
    return overlay;
  }

  applyInitialPosition(overlay, options) {
    if (options?.position && Number.isFinite(options.position.x) && Number.isFinite(options.position.y)) {
      this.applyPosition(overlay, options.position);
      return;
    }
    const width = this.container?.clientWidth || window.innerWidth || 0;
    const offset = (this.spawnIndex++ % 6) * 32;
    const maxLeft = Math.max(EDGE_PADDING, width - EDGE_PADDING - MIN_PANEL_WIDTH);
    const desiredLeft = width - MIN_PANEL_WIDTH - EDGE_PADDING - offset;
    const defaultLeft = clamp(desiredLeft, EDGE_PADDING, maxLeft);
    const top = EDGE_PADDING + offset;
    overlay.root.style.left = `${Math.max(EDGE_PADDING, defaultLeft)}px`;
    overlay.root.style.top = `${Math.max(EDGE_PADDING, top)}px`;
  }

  applyPosition(overlay, position) {
    const left = Number.isFinite(position.x) ? position.x : EDGE_PADDING;
    const top = Number.isFinite(position.y) ? position.y : EDGE_PADDING;
    overlay.root.style.left = `${left}px`;
    overlay.root.style.top = `${top}px`;
  }

  bringToFront(overlay) {
    if (!overlay?.root) {
      return;
    }
    this.zIndexSeed += 1;
    overlay.root.style.zIndex = String(this.zIndexSeed);
  }

  renderContent(overlay, content) {
    if (!overlay) {
      return;
    }
    const titleText = content?.title ?? '';
    overlay.titleEl.textContent = titleText;
    overlay.titleEl.classList.toggle('overlay-panel__title--empty', !titleText);

    const hasSubtitle = Boolean(content?.subtitle);
    overlay.subtitleEl.textContent = hasSubtitle ? content.subtitle : '';
    overlay.subtitleEl.hidden = !hasSubtitle;

    overlay.bodyEl.innerHTML = '';
    if (!content) {
      return;
    }

    if (content.description) {
      const descriptionLines = Array.isArray(content.description) ? content.description : [content.description];
      descriptionLines.forEach(line => {
        const p = document.createElement('p');
        p.className = 'overlay-panel__description';
        p.textContent = line;
        overlay.bodyEl.appendChild(p);
      });
    }

    const sections = Array.isArray(content.sections) ? content.sections : [];
    sections.forEach(section => {
      if (section.type === 'table') {
        this.renderTableSection(overlay.bodyEl, section);
      } else if (section.type === 'text') {
        this.renderTextSection(overlay.bodyEl, section);
      } else if (section.type === 'code') {
        this.renderCodeSection(overlay.bodyEl, section);
      }
    });
  }

  renderTextSection(container, section) {
    const wrapper = document.createElement('div');
    wrapper.className = 'overlay-panel__section';
    if (section.title) {
      const heading = document.createElement('div');
      heading.className = 'overlay-panel__section-title';
      heading.textContent = section.title;
      wrapper.appendChild(heading);
    }
    const lines = Array.isArray(section.text) ? section.text : [section.text];
    lines.forEach(line => {
      const p = document.createElement('p');
      p.textContent = line;
      wrapper.appendChild(p);
    });
    container.appendChild(wrapper);
  }

  renderTableSection(container, section) {
    const wrapper = document.createElement('div');
    wrapper.className = 'overlay-panel__section';
    if (section.title) {
      const heading = document.createElement('div');
      heading.className = 'overlay-panel__section-title';
      heading.textContent = section.title;
      wrapper.appendChild(heading);
    }
    const table = document.createElement('table');
    table.className = 'overlay-panel__table';
    if (section.headers && section.headers.length) {
      const thead = document.createElement('thead');
      const row = document.createElement('tr');
      section.headers.forEach(text => {
        const th = document.createElement('th');
        th.textContent = text;
        row.appendChild(th);
      });
      thead.appendChild(row);
      table.appendChild(thead);
    }
    const tbody = document.createElement('tbody');
    const rows = Array.isArray(section.rows) ? section.rows : [];
    rows.forEach(rowData => {
      const tr = document.createElement('tr');
      const cells = Array.isArray(rowData.cells) ? rowData.cells : rowData;
      cells.forEach(cell => {
        const td = document.createElement('td');
        td.textContent = cell ?? '';
        tr.appendChild(td);
      });
      if (rowData.action && this.onAction) {
        tr.classList.add('overlay-panel__row--actionable');
        tr.addEventListener('click', () => {
          this.onAction?.(rowData.action);
        });
      }
      tbody.appendChild(tr);
    });
    table.appendChild(tbody);
    wrapper.appendChild(table);
    container.appendChild(wrapper);
  }

  renderCodeSection(container, section) {
    const wrapper = document.createElement('div');
    wrapper.className = 'overlay-panel__section';
    if (section.title) {
      const heading = document.createElement('div');
      heading.className = 'overlay-panel__section-title';
      heading.textContent = section.title;
      wrapper.appendChild(heading);
    }
    const pre = document.createElement('pre');
    pre.className = 'overlay-panel__code';
    pre.textContent = formatCode(section.code);
    wrapper.appendChild(pre);
    container.appendChild(wrapper);
  }

  syncSize(overlay) {
    if (!overlay?.root?.isConnected) {
      return;
    }
    const containerWidth = this.container?.clientWidth || window.innerWidth || MIN_PANEL_WIDTH;
    const containerHeight = this.container?.clientHeight || window.innerHeight || 0;
    const maxWidth = clamp(containerWidth - EDGE_PADDING * 2, MIN_PANEL_WIDTH, MAX_PANEL_WIDTH);
    overlay.root.style.maxWidth = `${maxWidth}px`;
    overlay.root.style.maxHeight = `${Math.max(160, containerHeight - EDGE_PADDING * 2)}px`;
    overlay.root.style.width = 'auto';
    const measured = overlay.root.scrollWidth + 24;
    const width = clamp(Math.ceil(measured), MIN_PANEL_WIDTH, maxWidth);
    overlay.root.style.width = `${width}px`;
  }

  ensureWithinBounds(overlay) {
    if (!overlay?.root?.isConnected) {
      return;
    }
    const width = this.container?.clientWidth || window.innerWidth || 0;
    const height = this.container?.clientHeight || window.innerHeight || 0;
    if (width === 0 || height === 0) {
      return;
    }
    const panelWidth = overlay.root.offsetWidth;
    const panelHeight = overlay.root.offsetHeight;
    const maxLeft = Math.max(EDGE_PADDING, width - panelWidth - EDGE_PADDING);
    const maxTop = Math.max(EDGE_PADDING, height - panelHeight - EDGE_PADDING);
    const currentLeft = this.resolvePositionValue(overlay.root.style.left, overlay.root.offsetLeft);
    const currentTop = this.resolvePositionValue(overlay.root.style.top, overlay.root.offsetTop);
    overlay.root.style.left = `${clamp(currentLeft, EDGE_PADDING, maxLeft)}px`;
    overlay.root.style.top = `${clamp(currentTop, EDGE_PADDING, maxTop)}px`;
  }

  resolvePositionValue(styleValue, fallback) {
    const numeric = parseFloat(styleValue);
    if (Number.isFinite(numeric)) {
      return numeric;
    }
    if (Number.isFinite(fallback)) {
      return fallback;
    }
    return EDGE_PADDING;
  }

  startDrag(event, overlay) {
    if (!overlay?.root) {
      return;
    }
    if (event.button !== undefined && event.button !== 0) {
      return;
    }
    if (event.target.closest('button')) {
      return;
    }
    event.preventDefault();
    const containerRect = this.container.getBoundingClientRect();
    const overlayRect = overlay.root.getBoundingClientRect();
    this.dragState = {
      overlayId: overlay.id,
      pointerId: event.pointerId,
      offsetX: event.clientX - overlayRect.left,
      offsetY: event.clientY - overlayRect.top,
      containerLeft: containerRect.left,
      containerTop: containerRect.top,
    };
    this.bringToFront(overlay);
    window.addEventListener('pointermove', this.handlePointerMove);
    window.addEventListener('pointerup', this.handlePointerUp);
    window.addEventListener('pointercancel', this.handlePointerUp);
  }

  handlePointerMove(event) {
    if (!this.dragState || event.pointerId !== this.dragState.pointerId) {
      return;
    }
    event.preventDefault();
    const overlay = this.overlays.get(this.dragState.overlayId);
    if (!overlay?.root) {
      return;
    }
    const width = this.container?.clientWidth || window.innerWidth || 0;
    const height = this.container?.clientHeight || window.innerHeight || 0;
    const panelWidth = overlay.root.offsetWidth;
    const panelHeight = overlay.root.offsetHeight;
    const maxLeft = Math.max(EDGE_PADDING, width - panelWidth - EDGE_PADDING);
    const maxTop = Math.max(EDGE_PADDING, height - panelHeight - EDGE_PADDING);
    const left = clamp(
      event.clientX - this.dragState.containerLeft - this.dragState.offsetX,
      EDGE_PADDING,
      maxLeft,
    );
    const top = clamp(
      event.clientY - this.dragState.containerTop - this.dragState.offsetY,
      EDGE_PADDING,
      maxTop,
    );
    overlay.root.style.left = `${left}px`;
    overlay.root.style.top = `${top}px`;
  }

  handlePointerUp(event) {
    if (!this.dragState || event.pointerId !== this.dragState.pointerId) {
      return;
    }
    this.endDrag();
  }

  endDrag() {
    if (!this.dragState) {
      return;
    }
    window.removeEventListener('pointermove', this.handlePointerMove);
    window.removeEventListener('pointerup', this.handlePointerUp);
    window.removeEventListener('pointercancel', this.handlePointerUp);
    this.dragState = null;
  }

  handleWindowResize() {
    this.overlays.forEach(overlay => {
      this.syncSize(overlay);
      this.ensureWithinBounds(overlay);
    });
  }

  generateId() {
    return `overlay-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
  }
}

function formatCode(value) {
  if (typeof value === 'string') {
    return value;
  }
  try {
    return JSON.stringify(value, null, 2);
  } catch (error) {
    return String(value);
  }
}
