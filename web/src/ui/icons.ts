// The game's icons: inline SVG, stroked in currentColor.

const svg = (body: string, size = 22) =>
  `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">${body}</svg>`;

export const icons = {
  logo: `<svg viewBox="0 0 48 48" fill="none" stroke-width="3"><circle cx="12" cy="12" r="7" stroke="#FF6B35"/><circle cx="36" cy="12" r="7" stroke="#FF6B35"/><circle cx="12" cy="36" r="7" stroke="#FF6B35"/><circle cx="36" cy="36" r="7" stroke="#FF6B35"/><path d="M17 17l14 14M31 17L17 31" stroke="#22C7B8" stroke-width="4" stroke-linecap="round"/></svg>`,
  freefly: svg(
    '<path d="M3 12c3-5 6-7 9-7s6 2 9 7c-3 5-6 7-9 7s-6-2-9-7z"/><circle cx="12" cy="12" r="2.5"/>',
  ),
  duel: svg(
    '<path d="M4 20l7-7M20 20l-7-7M14.5 4.5L20 4l-.5 5.5L13 16M9.5 4.5L4 4l.5 5.5L11 16"/>',
  ),
  race: svg('<path d="M4 21V4M4 4h13l-2 4 2 4H4"/>'),
  options: svg(
    '<circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.7 1.7 0 0 0 .3 1.8l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.7 1.7 0 0 0-1.8-.3 1.7 1.7 0 0 0-1 1.5V21a2 2 0 1 1-4 0v-.1a1.7 1.7 0 0 0-1.1-1.5 1.7 1.7 0 0 0-1.8.3l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1a1.7 1.7 0 0 0 .3-1.8 1.7 1.7 0 0 0-1.5-1H3a2 2 0 1 1 0-4h.1a1.7 1.7 0 0 0 1.5-1.1 1.7 1.7 0 0 0-.3-1.8l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1a1.7 1.7 0 0 0 1.8.3H9a1.7 1.7 0 0 0 1-1.5V3a2 2 0 1 1 4 0v.1a1.7 1.7 0 0 0 1 1.5 1.7 1.7 0 0 0 1.8-.3l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1a1.7 1.7 0 0 0-.3 1.8V9a1.7 1.7 0 0 0 1.5 1H21a2 2 0 1 1 0 4h-.1a1.7 1.7 0 0 0-1.5 1z"/>',
  ),
  multirotor: svg(
    '<circle cx="6" cy="6" r="3"/><circle cx="18" cy="6" r="3"/><circle cx="6" cy="18" r="3"/><circle cx="18" cy="18" r="3"/><path d="M8 8l8 8M16 8l-8 8"/>',
  ),
  fixedwing: svg('<path d="M12 3l9 13-9-3-9 3z"/><path d="M12 13v8"/>'),
  ground: svg(
    '<rect x="2" y="9" width="14" height="7" rx="1"/><path d="M16 11h3l3 3v2h-6"/><circle cx="6" cy="18" r="2"/><circle cx="17" cy="18" r="2"/>',
  ),
  back: svg('<path d="M15 18l-6-6 6-6"/>', 18),
  pin: svg(
    '<path d="M12 21s-7-6.5-7-12a7 7 0 0 1 14 0c0 5.5-7 12-7 12z"/><circle cx="12" cy="9" r="2.5"/>',
    16,
  ),
  wind: svg('<path d="M3 8h11a3 3 0 1 0-3-3M3 12h16a3 3 0 1 1-3 3M3 16h8"/>', 16),
  video: svg('<rect x="2" y="6" width="14" height="12" rx="2"/><path d="M16 10l6-3v10l-6-3"/>', 16),
  eye: svg(
    '<path d="M2 12s4-7 10-7 10 7 10 7-4 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>',
    16,
  ),
  controller: svg(
    '<rect x="2" y="7" width="20" height="12" rx="4"/><path d="M7 11v4M5 13h4"/><circle cx="16" cy="12" r="1"/><circle cx="18" cy="14" r="1"/>',
    16,
  ),
};
