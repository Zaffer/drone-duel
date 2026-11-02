// A tiny DOM helper, and effects that tie a node's life to the signals it reads (DESIGN.md §9).
// No framework: `h()` builds nodes, `bind()` runs an effect until the node's scope is disposed.

import { effect, effectScope, setActiveSub } from "alien-signals";

type Child = Node | string | number | null | undefined | false | Child[];
type Attrs = Record<string, unknown> & {
  class?: string;
  style?: string | Partial<CSSStyleDeclaration>;
};

export function h<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  attrs?: Attrs | null,
  ...children: Child[]
): HTMLElementTagNameMap[K] {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs ?? {})) {
    if (v === undefined || v === null || v === false) continue;
    if (k === "style" && typeof v === "object") Object.assign(el.style, v);
    else if (k.startsWith("on") && typeof v === "function") {
      el.addEventListener(k.slice(2).toLowerCase(), v as EventListener);
    } else if (k in el && k !== "list" && typeof v !== "string") {
      (el as unknown as Record<string, unknown>)[k] = v;
    } else el.setAttribute(k, v === true ? "" : String(v));
  }
  append(el, children);
  return el;
}

function append(el: Node, children: Child[]): void {
  for (const c of children) {
    if (c === null || c === undefined || c === false) continue;
    if (Array.isArray(c)) append(el, c);
    else el.appendChild(typeof c === "object" ? c : document.createTextNode(String(c)));
  }
}

/** Replace `el`'s children. */
export function render(el: Element, ...children: Child[]): void {
  el.replaceChildren();
  append(el, children);
}

/** A scope of effects: dispose them all at once (a screen or panel going away). */
export function scope(fn: () => void): () => void {
  return effectScope(fn);
}

/** Run `fn` without the effect round it tracking what `fn` reads (or the effects it makes). */
export function untracked<T>(fn: () => T): T {
  const prev = setActiveSub(undefined);
  try {
    return fn();
  } finally {
    setActiveSub(prev);
  }
}

/** Run `fn` now and whenever a signal it reads changes. */
export function bind(fn: () => void): () => void {
  return effect(fn);
}
