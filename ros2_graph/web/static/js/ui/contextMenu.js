import { toggleHidden } from '../utils/dom.js';

export class ContextMenu {
  constructor(element, { getItems, onSelect }) {
    this.element = element;
    this.getItems = getItems;
    this.onSelect = onSelect;
    this.visible = false;
    this.target = null;
    this.items = [];

    if (this.element) {
      this.element.addEventListener('click', event => {
        const action = event.target?.dataset?.action;
        if (!action) {
          return;
        }
        event.stopPropagation();
        this.onSelect?.(action, this.target);
        this.toggle(false);
      });
    }
  }

  contains(node) {
    if (!this.element) {
      return false;
    }
    return this.element.contains(node);
  }

  toggle(visible, point, target) {
    if (!this.element) {
      return;
    }
    if (!visible) {
      this.visible = false;
      this.target = null;
      this.items = [];
      toggleHidden(this.element, true);
      this.element.classList.remove('visible');
      return;
    }
    const items = this.getItems ? this.getItems(target) : [];
    if (!items.length) {
      this.toggle(false);
      return;
    }
    this.element.innerHTML = '';
    items.forEach(item => {
      const button = document.createElement('button');
      button.type = 'button';
      button.dataset.action = item.action;
      button.textContent = item.label;
      this.element.appendChild(button);
    });
    this.items = items;
    this.target = target;
    this.visible = true;
    if (point) {
      const { x, y } = point;
      this.element.style.left = `${Math.round(x)}px`;
      this.element.style.top = `${Math.round(y)}px`;
    }
    toggleHidden(this.element, false);
    this.element.classList.add('visible');
  }
}
